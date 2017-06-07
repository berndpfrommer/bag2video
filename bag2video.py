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
    times.append(msg.header.stamp.to_sec())
    ntopic = len(topic)
    size = (msg.width * len(topic), msg.height)
    if ntopic % 2 == 0:
        size = (msg.width * 2, (int)(msg.height * ntopic/2 + 1e-8))

    # now read the rest of the messages for the rates
    iterator = bag.read_messages(topics=topic[0], start_time=start_time, end_time=stop_time)#, raw=True)
    for _, msg, _ in iterator:
        time = msg.header.stamp
        times.append(time.to_sec())
    tick = 1000
    fps = round(len(times)*tick/(max(times)-min(times)))/tick
    return fps, size, times

def calc_n_frames(times, precision=10):
    # the smallest interval should be one frame, larger intervals more
    intervals = np.diff(times)
    return np.int64(np.round(precision*intervals/min(intervals)))

def handle_audio_packet(mp3file, time, msg):
    mp3file.write("".join(msg.data))

def drain_buffer(tmax, writer, topics, total, count, frame_buffer, last_img):
    t = min(frame_buffer)
    #print "draining buffer: %f -> %f diff %f" % (t, tmax, tmax - t)
    while t <= tmax and len(frame_buffer) > 0:
        t = min(frame_buffer)
        # write the frame if we have an image from each camera
        count += 1
        if len(frame_buffer[t]) == len(topics):
            if count < total:
                # stack frames for all topics
                if len(topics) % 2 != 0:
                    wide_img = np.hstack([frame_buffer[t][tp] for tp in topics])
                else:
                    imgleft = np.vstack([frame_buffer[t][tp] for tp in topics[0::2]])
                    imgright = np.vstack([frame_buffer[t][tp] for tp in topics[1::2]])
                    wide_img = np.hstack([imgleft, imgright])
                print '\rWriting frame %s of %s at time %.6f' % (count-1, total, t)
                writer.write(wide_img)
                last_img = wide_img
        # we don't have images from each camera, write the last one again!
        else:
            if not (last_img is None):
                print '\rFILLING IN FRAME %s of %s at time %.6f' % (count-1, total, t)
                writer.write(last_img)
        # either way, it's time to delete this frame buffer
        del frame_buffer[t]
    return count, frame_buffer, last_img
    

def handle_video_packet(writer, bridge, total, viz, topics, topic, msg, time, last_time, count, frame_buffer, last_img):
    tstamp = msg.header.stamp.to_sec();
    #print "frame: %s time: %.6f" % (topic, tstamp)
    img = np.asarray(bridge.imgmsg_to_cv2(msg, 'bgr8'))
    if not (tstamp in frame_buffer):
        frame_buffer[tstamp] = {}
    frame_buffer[tstamp][topic] = img
    found_complete_frame = (len(frame_buffer[tstamp]) == len(topics))
    count, frame_buffer, last_img = drain_buffer(max(frame_buffer) - 2.0, writer,
                                                 topics, total, count, frame_buffer, last_img)
    return last_time, count, found_complete_frame, frame_buffer, last_img

def write_frames(bag, writer, total, topics=None, start_time=rospy.Time(0), stop_time=rospy.Time(sys.maxint), viz=False, encoding='bgr8'):
    bridge = CvBridge()
    if viz:
        cv2.namedWindow('win')
    count = 0
    video_topics = get_video_topics(topics)
    iterator = bag.read_messages(topics=topics, start_time=start_time, end_time=stop_time)
    last_time = -1
    frames = {}
    mp3file = open("audio.mp3", 'w')
    video_frame_found = False
    frame_buffer = {}
    last_img = None
    for (topic, msg, time) in iterator:
        if msg._type == 'audio_common_msgs/AudioData':
            if video_frame_found:
                handle_audio_packet(mp3file, time, msg)
        else:
            last_time, count, found_complete_frame, frame_buffer, last_img = handle_video_packet(writer, bridge, total, viz,
                                                                                                 video_topics, topic, msg, time,
                                                                                                 last_time, count, frame_buffer, last_img)
            if found_complete_frame:
                video_frame_found = True

    drain_buffer(max(frame_buffer), writer, video_topics, total, count, frame_buffer, last_img)
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
#            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.avi'
            outfile = os.path.join(*os.path.split(bagfile)[-1].split('.')[:-1]) + '.mp4'
        outfile_tmp = "tmp_" + outfile;
        bag = rosbag.Bag(bagfile, 'r')
        print 'Calculating video properties'
        topics = args.topic.split(",")
        video_topics = get_video_topics(topics)
        fps, size, times = get_info(bag, video_topics, start_time=args.start, stop_time=args.end)
        # writer = cv2.VideoWriter(outfile_tmp, cv2.cv.CV_FOURCC(*'DIVX'), fps, size)
        print 'Writing video to file %s with rate %.10f/sec' % (outfile_tmp, fps)
        
        #writer = cv2.VideoWriter(outfile_tmp, cv2.VideoWriter_fourcc(*'DIVX'), fps, size)
        writer = cv2.VideoWriter(outfile_tmp, cv2.VideoWriter_fourcc(*'X264'), fps, size)
        write_frames(bag, writer, len(times), topics=topics,
                     start_time=args.start, stop_time=args.end, viz=args.viz, encoding=args.encoding)
        writer.release()
        print '\n'
        print 'Adding audio'
        os.system("ffmpeg -i " + outfile_tmp + " -i audio.mp3 -codec copy -shortest " + outfile)
    
