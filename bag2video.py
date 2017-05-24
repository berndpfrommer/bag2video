#!/usr/bin/env python

from __future__ import division
import rosbag, rospy, numpy as np
import sys, os, cv2, glob
from itertools import izip, repeat
from subprocess import call
import argparse

# try to find cv_bridge:
try:
    from cv_bridge import CvBridge
except ImportError:
    # assume we are on an older ROS version, and try loading the dummy manifest
    # to see if that fixes the import error
    try:
        import roslib; roslib.load_manifest("bag2video")
        from cv_bridge import CvBridge
    except:
        print "Could not find ROS package: cv_bridge"
        print "If ROS version is pre-Groovy, try putting this package in ROS_PACKAGE_PATH"
        sys.exit(1)

def get_video_topics(topics):
    return [t for t in topics if t != "/audio/audio"];

def get_info(bag, topic=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxint)):
    size = (0,0)
    times = []

    # read the first message to get the image size
    msg = bag.read_messages(topics=topic[0]).next()[1]
    ntopic = len(topic)
    size = (msg.width * len(topic), msg.height)
    if ntopic % 2 == 0:
        size = (msg.width * 2, (int)(msg.height * ntopic/2 + 1e-8))

    # now read the rest of the messages for the rates
    iterator = bag.read_messages(topics=topic[0], start_time=start_time, end_time=stop_time)#, raw=True)
    for _, msg, _ in iterator:
        time = msg.header.stamp
        times.append(time.to_sec())
    diffs = 1/np.diff(times)
    return np.median(diffs), min(diffs), max(diffs), size, times

def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision*intervals/min(intervals)))

def handle_audio_packet(mp3file, time, msg):
    mp3file.write("".join(msg.data))

def handle_video_packet(writer, bridge, total, viz, topics, nframes, topic, msg, time, last_time, count, frames):
    tstamp = msg.header.stamp
    img = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
    frame_written = False
    if (tstamp != last_time): # new frame started
        frames = {} # clear out incomplete frames
        last_time = tstamp
        count += 1
    frames[topic] = img # remember frame for this topic
    if len(frames) == len(topics) and count < len(nframes): # have frames from all topics
        frame_written = True
        # stack frames for all topics
        if len(topics) % 2 != 0:
            wide_img = np.hstack([frames[t] for t in topics])
        else:
            imgleft = np.vstack([frames[t] for t in topics[0::2]])
            imgright = np.vstack([frames[t] for t in topics[1::2]])
            wide_img = np.hstack([imgleft, imgright])
        frames = {}
        print '\rWriting frame %s of %s at time %s' % (count-1, total, tstamp)
        for rep in range(nframes[count-1]):
            writer.write(wide_img)
        if viz:
            imshow('win', wide_img)
    return last_time, count, frames, frame_written

def write_frames(bag, writer, total, topics=None, nframes=repeat(1), start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxint), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    if viz:
        cv2.namedWindow('win')
    count = 0
    video_topics = get_video_topics(topics)
    iterator = bag.read_messages(topics=topics, start_time=start_time, end_time=stop_time)
    last_time = -1
    frames = {}
    mp3file = open("audio.mp3", 'w');
    video_frame_found = False
    for (topic, msg, time) in iterator:
        if msg._type == 'audio_common_msgs/AudioData':
            if video_frame_found:
                handle_audio_packet(mp3file, time, msg)
        else:
            last_time, count, frames, frame_written = handle_video_packet(writer, bridge, total, viz,
                                video_topics, nframes, topic, msg, time, last_time, count, frames)
            if frame_written:
                video_frame_found = True
    mp3file.close()

def imshow(win, img):
    cv2.imshow(win, img)
    cv2.waitKey(1)

def noshow(win, img):
    pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Extract and encode video from bag files.')
    parser.add_argument('--outfile', '-o', action='store', default=None,
                        help='Destination of the video file. Defaults to the location of the input file.')
    parser.add_argument('--precision', '-p', action='store', default=1, type=int,
                        help='Precision of variable framerate interpolation. Higher numbers\
                        match the actual framerater better, but result in larger files and slower conversion times.')
    parser.add_argument('--viz', '-v', action='store_true', help='Display frames in a GUI window.')
    parser.add_argument('--start', '-s', action='store', default=rospy.Time(0), type=rospy.Time,
                        help='Rostime representing where to start in the bag.')
    parser.add_argument('--end', '-e', action='store', default=rospy.Time(sys.maxint), type=rospy.Time,
                        help='Rostime representing where to stop in the bag.')
    parser.add_argument('--encoding', choices=('rgb8', 'bgr8', 'mono8'), default='bgr8',
                        help='Encoding of the deserialized image.')

    parser.add_argument('topic')
    parser.add_argument('bagfile')

    args = parser.parse_args()

    if not args.viz:
        imshow = noshow

    for bagfile in glob.glob(args.bagfile):
        print bagfile
        outfile = args.outfile
        outfile = args.outfile
        if not outfile:
            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
        outfile_tmp = "tmp_" + outfile;
        bag = rosbag.Bag(bagfile, 'r')
        print 'Calculating video properties'
        topics = args.topic.split(",")
        video_topics = get_video_topics(topics)
        rate, minrate, maxrate, size, times = get_info(bag, video_topics, start_time=args.start, stop_time=args.end)
        nframes = calc_n_frames(times, args.precision)
        # writer = cv2.VideoWriter(outfile_tmp, cv2.cv.CV_FOURCC(*'DIVX'), rate, size)
        writer = cv2.VideoWriter(outfile_tmp, cv2.VideoWriter_fourcc(*'DIVX'), np.ceil(maxrate*args.precision), size)
        print 'Writing video'
        write_frames(bag, writer, len(times), topics=topics, nframes=nframes,
                     start_time=args.start, stop_time=args.end, viz=args.viz, encoding=args.encoding)
        writer.release()
        print '\n'
        print 'Adding audio'
        os.system("ffmpeg -i " + outfile_tmp + " -i audio.mp3 -codec copy -shortest " + outfile)
    
