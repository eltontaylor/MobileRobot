"""
A simple fixation task in which the monkey is rewarded for maintaining fixation at the central spot while targets are
flashed in the periphery.
"""
import sys
sys.path.insert(1, '/usr/local/lib/python2.7/dist-packages/')
from psychopy import visual, core, monitors, logging, event, tools, data, parallel
from matplotlib import pylab
from matplotlib.pylab import mlab
from psychopy import prefs
prefs.general["audioLib"] = ["pygame"]
from psychopy import sound
from pylinkwrapper import *
#import pylinkwrapper
import numpy as np
import numpy.lib.recfunctions
import itertools
import os
import json
import serial
import utils
import calibration
import traceback as tb

# define tiggers; for stimuli, the first two bits indicate the color (01 vs 10),
# the third bit indicate onset/offset
# (1/0), the last 5 bit indicate position. This gives us 32 possible locations,
# which should be more than sufficient
triggers = {"session_start": "11000000",
            "trial_start": "00000010",
            "fix_start": "00000001",
            "reward_on": "00000110",
            "reward_off": "00000100",
            "manual_reward_on": "00001100",
            "manual_reward_off": "00001000",
            "failure": "00000111",
            "left_fixation": "00011101",
            "trial_end": "00100000",
            "target_on": "10100000",
            "target_off": "10000000",
            "distractor_on": "01100000",
            "distractor_off": "01000000",
            "response_on": "00000101"}
plexon_triggers = dict([(k, int(v, 2)) for k, v in triggers.items()])
# setup parallel port
try:
    parallel_port = parallel.ParallelPort(0xD050)
    parallel_port.strobe_pin = 18
    #parallel_port.setPin(parallel_port.strobe_pin, 1)
    parallel_port.setData(255)  # set to zero

except:
    parallel_port = None
    print "Running without parallel port. No strobes will be recorded!"

#define colors
red = (1.0, 0.0, 0.0)
green = (0.0, 1.0, 0.0)
white = (1.0, 1.0, 1.0)
black = (-1.0, -1.0, -1.0)
blue = (-1.0, -1.0, 1.0)
yellow = (1.0, 1.0, -1.0)
colors = {"red": red,
          "green": green,
          "white": white,
          "black": black,
          "blue": blue,
          "yellow": yellow}

def create_strobe(marker, pos,width=8):
    vv = int(triggers[marker], 2)  # convert from binary to integer
    if 0 < pos < 32:
        vv = vv + pos
    return np.binary_repr(vv, width)  # convert back to binary


class Target():
    def __init__(self, duration, num_targets):
        self.duration = duration
        self.num_targets = num_targets
        self.onset_clock = core.Clock()
        self.off_clock = core.Clock()
        self.on = False
        self.off = False
        self.idx = 0
        self.pos_idx = 0  # Index into the current position matrix
        self.identity = 0  # target = 0; distractor = 1
        self.on_marker = "target_on"
        self.off_marker = "target_off"
        self.active = True
        self.active_this_trial = False
        self.pending = False

    def update_marker(self):
        if self.identity == 0:
            self.on_marker = "target_on"
            self.off_marker = "target_off"
        elif self.identity == 1:
            self.on_marker = "distractor_on"
            self.off_marker = "distractor_off"
        else:
            self.on_marker = "blank_on"
            self.off_marker = "blank_off"

    def turn_on(self):
        self.on = True
        self.off = False
        self.active_this_trial = True
        self.off_clock.reset()

    def turn_off(self):
        self.on = False
        self.off = True

    def send_message(self, tracker, marker, clock, timedict):
        if self.identity < 2:  # do not send any markers for blanks
            trigger = create_strobe(marker, self.pos_idx)
            timedict[marker] = clock.getTime()
            if tracker is not None:
                tracker.send_message(trigger)
                send_strobe(int(trigger, 2))

    def show(self, tracker, clock, timedict, *reset_clocks):
        if self.active:
            self.send_message(tracker, self.on_marker, clock, timedict)
            self.turn_on()
            for _clock in reset_clocks:
                _clock.reset()

    def hide(self, tracker, clock ,timedict,*reset_clocks):
        self.send_message(tracker, self.off_marker, clock, timedict)
        self.turn_off()
        self.idx += 1
        if self.idx < self.num_targets:
            self.pending = True  # we still have more targets to present
        else:
            self.pending = False  # no more targets to present
        for _clock in reset_clocks:
            _clock.reset()


class Response():
    def __init__(self, duration):
        self.duration = duration
        self.clock = core.Clock()
        self.on = False
        self.on_marker = "response_on"

    def show(self, tracker, clock, timedict,*reset_clocks):
        send_message(tracker, self.on_marker, clock, timedict)
        self.on = True
        for _clock in reset_clocks:
            _clock.reset()


class CriterionClock(core.Clock):
    def __init__(self, duration, *args, **kwargs):
        core.Clock.__init__(self, *args, **kwargs)
        self.duration = duration
        self.active = False

    def finished(self):
        isfinished = self.getTime() >= self.duration
        if isfinished:
            self.active = False
        return isfinished

    def reset(self):
        core.Clock.reset(self)
        self.active = True


def summary_plot(timestamps, manual_reward_duration, droprate=1.0, filename=None):
   pylab.style.use("bmh")
   ncorrect = (timestamps["reward_on"]>0).sum()
   idx = timestamps["failure"] > 0
   nfailure = idx.sum()
   nmissed = ((timestamps["failure"]==0)*(timestamps["reward_on"]==0)).sum()
   failure_time = (timestamps["failure"][idx]- timestamps["fix_start"][idx])*1000
   reward_amount = (timestamps["reward_off"] - timestamps["reward_on"]).sum()
   f = pylab.figure()
   ax1 = f.add_subplot(221)
   ax1.bar([1,2,3], [ncorrect, nfailure, nmissed])
   ax1.set_xticks([1.4,2.4,3.4])
   ax1.set_xticklabels(["correct","failure", "missed"])
   ax1.set_xlim(0.8	, 4.0)
   ax1.set_ylabel("Number of trials")

   ax2 = f.add_subplot(222)
   ax2.bar([1,2], [reward_amount*droprate, manual_reward_duration*droprate])
   ax2.set_xticks([1.4,2.4])
   ax2.set_xlim(0.8	, 3.0)
   ax2.set_xticklabels(["Reward","Manual reward"])
   ax2.set_ylabel("Liquid amount [ml]")

   ax3 = f.add_subplot(212)
   ax3.hist(failure_time)
   ax3.set_xlabel("Time to failure [ms]")
   pylab.tight_layout()
   if filename is None:
       pylab.show()
   else:
       f.savefig(filename)


def get_gaze(tracker):
    if tracker.realconnect:
            # Grab latest sample
            sample = tracker.tracker.getNewestSample()
            if sample is None:
                # Check if sample is a nonetype object then assign gaze = nan
                gaze = [np.nan, np.nan]
            else:
               # Extract gaze coordinates
                if sample.isRightSample():
                    gaze = sample.getRightEye().getGaze()
                else:
                    gaze = sample.getLeftEye().getGaze()

            return gaze
    else:
        return [np.nan, np.nan]


def create_bullseye(N=256,k=128):
    m = N/2
    x,y = np.meshgrid(np.arange(N), np.arange(N))
    Z = np.cos(2 * np.pi / k * np.sqrt((x - m) ** 2 + (y - m) ** 2))
    return Z


def send_strobe(data):
    """
    Send data as a strobe to the plexon system.
    :param data:  16 bit integer representing the binary strobe to be sent
    :return:  nothing
    """
    if parallel_port is not None:
        # the strobe pin is the first bit of the 3rd byte, i.e. bit
        # add two because setPin subtracts 2 from the given value
        strobe_pin = 18
        parallel_port.setData(data)  # need to invert the data
        core.wait(1e-4)  # need to wait at least 100 microseconds  before setting the strobe
        parallel_port.setPin(strobe_pin,0)  # activate the strobe pin


def send_message(tracker, marker, clock, timedict, idx=-1):
    if tracker is not None:
        tracker.send_message(triggers[marker])
    timedict[marker] = clock.getTime()
    if idx > -1:
        strobe = int(create_strobe(marker, idx),2)  # add idx to strobe
    else:
        strobe = plexon_triggers[marker]
    send_strobe(strobe)


def target_preview(exp_info):
    target_locations = exp_info.get("target_locations",[])
    if target_locations:
        screen_width = 800.0
        screen_height = 600.0
        #scale targets since the previe window is smaller than the actual window
        scale_x = screen_width/exp_info.get("screen_width",1650)
        scale_y = screen_height/exp_info.get("screen_height",1080)

        win = visual.Window(
            size=(screen_width, screen_height), fullscr=False, screen=0,
            allowGUI=True, allowStencil=False,viewScale=[scale_x, scale_y],
            monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
            blendMode='avg', useFBO=False)

        target_color = exp_info.get("target_color", red)
        targets = visual.ElementArrayStim(win, units='deg', nElements=len(target_locations),
                                          sizes=exp_info.get("target_size",2.0),colors=target_color,
                                       colorSpace='rgb', sfs=0.0,xys=target_locations,elementMask=None)
        anchors = create_anchors(win, exp_info)
        while True:
            keys = event.getKeys()
            if "escape" in keys:
                break
            targets.draw()
            anchors.draw()
            win.flip()
        win.close()


def create_anchors(win, exp_info):
    #create a cross
    target_locations = exp_info.get("target_locations",[])
    anchor_text = -np.ones((256,256))
    idx1 = np.arange(127-10, 127+10)
    anchor_text[idx1[:,None], :] = 1
    anchor_text[:, idx1[None,:]] = 1
    anchors = visual.ElementArrayStim(win, units="deg", nElements=len(target_locations),
                                      sizes=exp_info.get("target_size", 1.0),
                                      sfs=0.0, oris=0.0, xys=target_locations,
                                      elementMask=anchor_text,colors=white, colorSpace='rgb')
    return anchors


def run_experiment(exp_info={},cresults={}):
    subject = exp_info.get("subject", "r")
    session = exp_info.get("session", 1)
    if not "date" in exp_info.keys():
        date = data.getDateStr('%Y%m%d')
        exp_info["date"] = date
    else:
        date = exp_info["date"]

    day = exp_info.get("day", data.getDateStr("%d")).lstrip("0")
    month = exp_info.get("month", data.getDateStr("%m")).lstrip("0")
    filename = '%s%s_%s_%d' % (subject[:2], month, day, session)
    if len(filename) > 8:
        return {}, "File name is longer than 8 characters"

    #save the triggers used
    exp_info["triggers"] = triggers
    #save the settings
    json.dump(exp_info, open(filename + "_settings.txt","w"))

    reward_duration = exp_info.get("reward_duration", 0.5)
    base_reward_duration = exp_info.get("base_reward_duration", 0.5)
    current_reward_duration = reward_duration
    scale_reward = exp_info.get("scale_reward", False)
    failure_distr = exp_info.get("failure_distr", {"nn": 0})
    #reward through serial
    serialpath = exp_info.get("serial_port", "")
    reward_cnx = utils.Reward(serialpath, reward_duration)
    reward_clock = core.Clock()
    reward_color = exp_info.get("reward_color", None)
    if reward_color == "green":
        reward_color = green
    manual_reward_clock = core.Clock()
    manual_reward_duration = exp_info.get("manual_reward_duration", 0.5)
    manual_reward_duration_2 = exp_info.get("manual_reward_duration_2", 0.5)
    manual_reward_duration_used = manual_reward_duration  # keep track of what type of manual reward we are using
    droprate = exp_info.get("droprate", 1.0)
    play_sound = exp_info.get("play_sound", False)
    if play_sound:
        reward_sound = sound.Sound('A')

    reward_total = cresults.get("reward_total", 0.0) #to keep track of how much reward was given by the proggram
    manual_reward_total = cresults.get("manual_reward_total", 0.0) #to keep track of how much manual reward was given
    in_reward = False
    in_manual_reward = False


    screen_width = exp_info.get("screen_width", 1680)
    screen_height = exp_info.get("screen_height", 1050)

    #setup window
    if sys.platform == 'darwin':
        fullscreen = False
    else:
        fullscreen = True

    win = visual.Window(
        size=(screen_width, screen_height), fullscr=fullscreen, screen=1,
        allowGUI=True, allowStencil=False,
        monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
        blendMode='avg', useFBO=False)

    show_second_window = exp_info.get("exp_plot", False)
    scale_x = 800/screen_width
    scale_y = 600/screen_height
    if show_second_window:
        win2 = visual.Window(
            size=(800, 600), fullscr=False, screen=0,
            allowGUI=True, allowStencil=False,
            monitor='testMonitor', color=[0,0,0], colorSpace='rgb',
            blendMode='avg', useFBO=False, viewScale=[scale_x, scale_y])

    frameinterval = win.monitorFramePeriod*0.75

    #setup monitor and screen using pygaze
    mon = win.monitor
    mon.setDistance(exp_info.get("screen_distance", 57))
    mon.setWidth(exp_info.get("screen_size", 22)*2.5)
    mon.setSizePix((screen_width,screen_height))
    win.setUnits('deg')

    fixation_size = exp_info.get("fixation_size", 2.0) #fixation size in degrees
    fixation_window_size = exp_info.get("fixation_window", 2.0)
    fixation_buffer = exp_info.get("fixation_buffer",0.0)
    fixation_mask = exp_info.get("fixation_mask",None)
    tex = None
    color = (1.0, 1.0, 1.0)
    contrast = 1.0
    if fixation_mask == "bullseye":
        tex = create_bullseye(256,128)
        fixation_mask = None
    elif fixation_mask == "square":
        fixation_mask = None
        tex = None

    mask_params = exp_info.get("mask_params", None)
    fixation_pos = [0.0, 0.0]
    fixation_spot = visual.GratingStim(win, size=fixation_size,units='deg',
                                    pos=fixation_pos, sf=0, color=color,contrast=contrast,
                                       maskParams=mask_params,mask=fixation_mask,
                                       tex=tex,interpolate=True,texRes=256)
    fixation_window = visual.GratingStim(win, size=fixation_window_size, units='deg',
                                         pos=[0,0], sf=0.0, mask=fixation_mask)

    if show_second_window:
        fixation_spot2 = visual.GratingStim(win2, size=fixation_size,units='deg',
                                           pos=fixation_pos, sf=0, color=color,contrast=contrast,
                                           maskParams=mask_params,mask=fixation_mask,
                                           tex=tex,interpolate=True,texRes=256)
        fixation_window2 = visual.GratingStim(win2, size=fixation_window_size, units='deg',
                                             pos=[0,0], sf=0.0, mask=fixation_mask,
                                              color=(0.7,0.7,0.7), colorSpace='rgb')

    #target stuff
    num_targets = exp_info.get("num_targets", 0)
    target_color = exp_info.get("target_color", red)
    target_onset = exp_info.get("target_onset", 0.0)
    target_duration = exp_info.get("target_duration", 1.0)
    inter_target_interval = exp_info.get("inter_target_interval", 0.0)
    inter_target_clock = core.Clock() # clock to keep track of delay between targets
    target_size = exp_info.get("target_size", 2.0)
    target_window_size = exp_info.get("target_window_size", target_size)
    target_locations = exp_info.get("target_locations")
    last_target_pos = (0.0, 0.0)  #auxillary variable to keep track of the last target
    if len(target_locations) == 0:
        target_locations.append((0.0, 0.0))
    n_target_locations = len(target_locations)
    target_contrast = exp_info.get("target_contrast", 1.0)
    target_mask = exp_info.get("target_mask", None)
    #FIXME: Set this from exp_info
    target_mask_params = None
    target_tex = None
    max_target_repeats = exp_info.get("repeat_failed_targets", 0)
    print max_target_repeats
    target = visual.GratingStim(win, size=target_size,units='deg',
                                       pos=target_locations[0], sf=0, color=target_color,contrast=target_contrast,
                                       maskParams=target_mask_params,mask=target_mask,
                                       tex=target_tex,interpolate=True,texRes=256)
    target_window = visual.GratingStim(win, size=target_window_size, units='deg',
                                         pos=target_locations[0], sf=0.0)

    target_timer = Target(target_duration, num_targets)
    target_onset_clock = core.Clock()
    target_onset_ref = 0
    if "target_onset_ref" in exp_info.keys():
        print exp_info["target_onset_ref"]
        if exp_info["target_onset_ref"] == "Trial start":
            target_onset_ref = 1
    target_fixation = (0.0, 0.0)  # current target fixation location
    n_target_fixation = 0  # current number of fixation steps
    target_presented = False  # flag to keep track of whether a target was presented

    response_timer = Response(0.0)

    show_anchors = exp_info.get("show_anchors", False)
    if show_anchors:
        anchors = create_anchors(win, exp_info)

    if show_second_window:
        target2 = visual.GratingStim(win2, size=target_size,units='deg',
                                    pos=target_locations[0], sf=0, color=target_color,contrast=target_contrast,
                                    maskParams=target_mask_params,mask=target_mask,
                                    tex=target_tex,interpolate=True,texRes=256)
        target_window2 = visual.GratingStim(win2, size=target_window_size, units='deg',
                                       pos=target_locations[0], sf=0.0)
    target_on_clock = core.Clock()
    target_off_clock = core.Clock()
    target_on = False


    eccentricity = exp_info.get("eccentricity", 5.0)
    fixation_locations = {"0": (0.0, 0.0),
                          "1" : (-eccentricity, eccentricity),
                          "2": (0.0, eccentricity),
                          "3": (eccentricity, eccentricity),
                          "4": (eccentricity, 0.0),
                          "5": (eccentricity, -eccentricity),
                          "6": (0.0, -eccentricity),
                          "7": (-eccentricity, -eccentricity),
                          "8": (-eccentricity, 0.0)}
    npositions = len(fixation_locations)
    autocycle = exp_info.get("autocycle", False) #whether to randomly cycle through positions


    eye_spot = visual.GratingStim(win, size=0.2,units='deg',
                                       pos=[0,0], sf=0, color=(-1.0, -1.0, -1.0),contrast=1.0)
    if show_second_window:
        eye_spot2 = visual.GratingStim(win2, size=0.2,units='deg',
                                  pos=[0,0], sf=0, color=(-1.0, -1.0, -1.0),contrast=1.0)
        fixation_mean_spot = visual.GratingStim(win2, size=0.2,units='deg',
                                       pos=[0,0], sf=0, color=red,contrast=1.0)

    #should we show the target
    show_fixation_spot = True
    fixation_criterion = exp_info.get("fixation_criterion", 1.0)
    fixation_grace = exp_info.get("fixation_grace", 0.3)
    #setup a clock to keep track of how long we have been inside the fixation spot
    in_fixation_clock = CriterionClock(fixation_criterion)
    left_fixation_clock = CriterionClock(fixation_grace) #keep track of how long the subject was outside fixation
    #variable to keep track of whether we are fixating
    in_fixation = False
    left_fixation = False
    #how long do we need to stay in fixation until we get rewarded

    #feedback clock to indicate that we are giving feedback to the monkey
    feedback_clock = core.Clock()
    if not ("feedback_duration" in exp_info.keys()):
        feedback_duration = exp_info["reward_duration"]
    else:
        feedback_duration = exp_info["feedback_duration"]
    in_feedback = False
    reward = False
    failure = False
    failure_color = exp_info.get("failure_color", None)
    if failure_color is not None:
        failure_color = colors[failure_color.lower()]

    #define an inter-trial period in which the monkey can do whatever he wants
    intertrial_clock = core.Clock()
    intertrial_duration = exp_info.get("intertrial_duration", 2.0)
    in_intertrial = True #define the period before the first trial as the first inter-trial period

    trial_timeout = exp_info.get("trial_timeout", 100.0) #maximum time for a trial

    #main clock
    experiment_clock = core.Clock()
    #trial clock
    trial_clock = core.Clock()
    #step clock
    step_clock = core.Clock()

    #variable to track whether we should continue the task
    continue_task = True

    #setup eye tracker
    use_dummy = exp_info.get("use_dummy", False)

    if use_dummy:
        tracker = None
        mouse = event.Mouse()
    else:
        tracker = Connect(window=win, edfname=filename + '.edf')
        #set up screen coordinates
        tracker.send_message("screen_pixel_coords = %d %d %d %d" %(0,  0, screen_width, screen_height))
        tracker.send_message("DISPLAY_COORDS %d %d %d %d" %(0, 0, screen_width, screen_height))
        #Should we calibrate?
        if exp_info.get("calibrate", True):
            #get the radius by dividing by 2
            calibration_stim = exp_info.get("calibration_stimulus")
            manual_calibration = exp_info.get("manual_calibration", False)
            pulse_dot = exp_info.get("Pulse", False) # whether to pulse the dot
            if calibration_stim == "Gabor patch":
                use_calibration_gabor = True
                calibration_image_path = None
            else:
                use_calibration_gabor = False
                if not (calibration_stim == "Circle"):
                    calibration_image_path = calibration_stim
                else:
                    calibration_image_path = None
            calibration_target_size = exp_info.get("calibration_target_size", 1.0)/2
            if (calibration_image_path is not None) or (use_calibration_gabor):
                """
                for circle, the size is given as a radius. For image or gabor patch, we need
                to multiple by two to get the diameter
                """
                calibration_target_size *= 2
            # convert from degress to pixels
            calibration_target_size = calibration.deg2pix(calibration_target_size,win.monitor)
            calibration_target_color = colors[exp_info.get("calibration_target_color", "white")]
            ctype = exp_info.get("calibration_type", "9 points")
            cnum = int(ctype.split()[0])
            # change reward duration during calibration
            calibration_reward_duration = exp_info.get("calibration_reward_duration", 0.5)
            reward_cnx.duration = calibration_reward_duration
            calibration_movie_path = None
            if calibration_image_path is not None:
                bn, ext = os.path.splitext(calibration_image_path)
                if ext in [".mp4"]:
                    calibration_movie_path = calibration_image_path
                    calibration_image_path = None
            calibration.calibrate(tracker, reward_cnx, cnum=cnum,
                                  target_color=calibration_target_color,
                                  target_size=calibration_target_size,
                                  target_image=calibration_image_path,
                                  use_gabor=use_calibration_gabor,
                                  pulse_dot=pulse_dot,
                                  manual_calibration=manual_calibration,
                                  movie_stim=calibration_movie_path)
            # change reward duration back
            reward_cnx.duration = reward_duration
        tracker.record_on(True)

    draw_eyespot = exp_info.get("draw_eyespot", False)
    t_mean = 0.0  # keep track of how long each trial last
    t_std = 0.0
    nframes = 0

    # keep track of the number of trials
    block_report = exp_info.get("block_report", False)
    if block_report:
        ntrials = 0
        ncorrect = 0
        nfailures = 0
        nmissed = 0
    else:
        ntrials = cresults.get("ntrials", 0)
        ncorrect = cresults .get("ncorrect", 0)
        nfailures = cresults.get("nfailures", 0)
        nmissed = cresults.get("nmissed", 0)

    # keep track of mean fixation position
    fix_distr = cresults.get("fix_distr", {})
    for k in fixation_locations.values():
        if not (k in fix_distr.keys()):
            fix_distr[k] = (k[0], k[1],0.0, 0.0, 0.0, 1)
    # keep track of mean delay for correct trials
    delay_mean = 0.0
    in_trial = False # indicate whether we are currently doing a trial

    make_response = exp_info.get("make_response", True)  # indicate whether the monkey should respond or just fixatte
    response_on_clock = CriterionClock(0.0)
    in_response_clock = core.Clock()
    response_cue_onset_min = exp_info.get("response_cue_onset_min", 0.0)
    response_cue_onset_max = exp_info.get("response_cue_onset_max", 0.0)
    response_cue_ref = 0  # target offset
    if exp_info.get("response_cue_ref", "Target onset") == "Target onset":
        response_cue_ref = 1  # target onset
    elif exp_info.get("response_cue_ref", "Target onset") == "Fixation":
        response_cue_ref = 2  # Use fulfilment of fixation as reference
    in_passive_response = False # keep track of whether we are waiting for response cue in a passive task
    max_rtime = exp_info.get("max_rtime", 0.0)
    max_saccade_time = exp_info.get("max_saccade_time", 0.0)
    target_fix_time = exp_info.get("target_fix_time", 0.0)

    in_target = False
    in_target_clock = core.Clock()

    #keep track of time
    timedict = {"reward_on": 0.0,
                "reward_off":0.0,
                "manual_reward_on":0.0,
                "manual_reward_off": 0.0,
                "failure":0.0,
                "trial_start":0.0,
                "trial_end":0.0,
                "fix_start":0.0,
                "target_on": 0.0,
                "target_off": 0.0,
                "left_fixation": 0.0,
                "response_on": 0.0}

    trial_start_times = []
    fix_start_times = []
    reward_times = []
    reward_off_times = []
    manual_reward_on_times = []
    manual_reward_off_times = []
    failure_times = []
    trial_end_times = []
    target_on_times = []
    target_off_times = []
    left_fixation_times = []
    response_on_times = []

    #collect target positions
    target_positions = []
    num_target_locations = len(target_locations)
    target_location_idx = np.arange(len(target_locations))

    #positions of the stimulus for each trial
    stimulus_positions = [(0,0) for i in xrange(num_targets)]
    #target = 0; distractor = 1, blank = 2
    num_stim_identities = 3
    stimulus_identity = np.zeros((num_targets,),dtype=np.int)
    #create all combinations of location and identity for each stimulus to be presented
    #create an array containing all combinations of locations and identities for num_targets
    #Note that the blank 'target' only occurs in one position (since it is not visible)
    #each element in params represents (id1, l1, id2, l2)
    #compute the total number of parameters per target
    #construct the parameters per target
    allowed_combos = exp_info.get("allowed_combos", [])
    # stim 1
    # if we only have a single stimulus, we only show the target
    p1 = []
    for l in xrange(num_target_locations):
        p1.append([(0, l)])

    # stim 2
    repeat_locations = exp_info.get("repeat_locations", False)
    if num_targets > 1:
        # if we have more than one stimulus, each stimulus can either be target, distractor or blank
        p1.append([(2, 0)])
        for l in xrange(num_target_locations):
            p1.append([(1, l)])
        for tt in xrange(1, num_targets):
            tparams = []
            for pp in p1:
                tparams.append(pp + [(2, 0)])  # blank
                for s in xrange(num_stim_identities-1):
                    for l in xrange(num_target_locations):
                        skip = False
                        for _p in pp:
                            if _p[1] == l:
                                skip = True
                                break
                        if not skip or repeat_locations:
                            tparams.append(pp + [(s, l)])
            p1 = list(tparams)
    else:
        tparams = p1
    if allowed_combos:
        stim_id_map = {1: "D", 0: "T", 2: "_"}
        params = []
        for pp in tparams:
            # filter out disallowed combos
            cc = ''.join([stim_id_map[_pp[0]] for _pp in pp])
            if cc in allowed_combos:
                params.append(pp)

    else:
        params = tparams
    if use_dummy:
        print "params = ", params
        print len(params)
    combo_probs = np.ones((len(params),))/len(params)
    all_stim_idx = np.arange(len(params))  # index into all variables
    np.random.shuffle(all_stim_idx)  # initial randomization
    combo_idx = 0  # index to keep track of the combinations presented
    n_target_repeats = 0  # counter to keep track of number of times a target is repeated
    #if num_targets > 1:
     #   stimulus_identity[-1] = 1

    stimulus_identities = []

    # collect fixation positions
    fixation_positions = []

    experiment_clock.reset()
    # start the main loop
    # send a strobe to indicate he start of the session
    send_message(tracker, "session_start", experiment_clock, timedict, session)
    is_paused = False  # flag to indicate that the task is currently paused
    try:
        while continue_task:
            step_clock.reset()
            # check if any keys were pressed
            keys = event.getKeys()
            if 'escape' in keys:  # escape
                continue_task = False
                if in_trial:
                    ntrials -= 1  # we aborted in the middle of a trial; exclude this
                                  # trial from the count
                break
                # This messes with the saved trial structure; the last trial
                # is never saved.
            elif 'r' in keys or 'l' in keys:
                # manual reward
                reward_cnx.open()
                if play_sound:
                    reward_sound.play()
                send_message(tracker, "manual_reward_on", experiment_clock, timedict)
                manual_reward_clock.reset()
                in_manual_reward = True
                if 'r' in keys:
                    manual_reward_duration_used = manual_reward_duration
                else:
                    manual_reward_duration_used = manual_reward_duration_2
            elif 'space' in keys:
                # pause task
                if is_paused:
                    is_paused = False
                else:
                    is_paused = True

            else:
                # check for number keys, which would indicate a change in fixation
                # position
                for k, pos in fixation_locations.items():
                    if k in keys:
                        fixation_pos = pos
                        break

            # check manual reward
            if (in_manual_reward and
                    manual_reward_clock.getTime() >= manual_reward_duration_used):
                in_manual_reward = False
                reward_cnx.close()
                if play_sound:
                    reward_sound.stop()
                send_message(tracker, "manual_reward_off", experiment_clock,
                             timedict)
                manual_reward_total += (timedict["manual_reward_off"] -
                                        timedict["manual_reward_on"])
            # check the eye position
            if use_dummy:
                x, y = mouse.getPos()
                eye_spot.setPos([x, y])
                if show_second_window:
                    eye_spot2.setPos([x, y])
            else:
                x, y = get_gaze(tracker)
                x, y = tracker.convert_coords(x, y, to='psychopy')
                eye_spot.setPos([x, y])
                if show_second_window:
                    eye_spot2.setPos([x, y])
            # check if the eye is inside the fixation window
            if fixation_window.contains(x, y):
                if not (in_fixation or in_feedback or in_intertrial or
                        response_timer.on or in_target):
                    timedict["fix_start"] = experiment_clock.getTime()
                    if use_dummy:
                        print "Acquired fixation at %f" % (timedict["fix_start"], )
                    else:
                        send_message(tracker, "fix_start", experiment_clock,
                                     timedict)
                    in_fixation = True
                    left_fixation = False
                    in_fixation_clock.reset()
            else:
                # not inside the fixation window
                if in_fixation:  # we just left fixation
                    # check whether the grace period has been exceeded
                    if in_fixation_clock.getTime() >= fixation_grace:
                        # check whether we just left the fixation window
                        if not left_fixation:
                            left_fixation_clock.reset()  # just left fixation;
                                                         # start the fixation buffer clock
                            left_fixation = True
                        # allow the monkey to leave the fixation for a brief period,
                        # deteremined by the fixation buffer
                        if (left_fixation and
                                left_fixation_clock.getTime() <= fixation_buffer):
                                in_fixation = True
                        else:
                            # fixation buffer time exceeded; the monkey is no
                            # longer fixating
                            in_fixation = False
                            if use_dummy:
                                timedict["left_fixation"] = experiment_clock.getTime()
                                print "Left fixation at %.2f" % (timedict["left_fixation"])
                            else:
                                send_message(tracker, "left_fixation",
                                             experiment_clock, timedict)
                    else:
                        in_fixation = False

            # check whether the monkey is fixating
            if in_fixation:
                #update fixation mean
                fixpos = tuple(fixation_spot.pos)
                fix_xm, fix_ym,fix_xs, fix_ys, fix_xys, fix_n = fix_distr[fixpos]
                #mean
                fix_xm = (fix_xm *fix_n + x)/(fix_n+1)
                fix_ym = (fix_ym *fix_n + y)/(fix_n+1)
                #std
                fix_xs = (fix_xs*fix_n + x*x)/(fix_n+1)
                fix_ys = (fix_ys*fix_n + y*y)/(fix_n+1)
                fix_xys = (fix_xys*fix_n + x*y)/(fix_n+1)
                fix_n += 1
                fix_distr[fixpos] = (fix_xm, fix_ym, fix_xs, fix_ys,
                                     fix_xys, fix_n)
                if show_second_window:
                    fixation_mean_spot.setPos((fix_xm, fix_ym))

                if in_fixation_clock.active and in_fixation_clock.finished():
                    if response_cue_ref == 2:  # respond after fixation
                        response_on_clock.reset()
                    elif (target_onset_ref == 0) and (not target_timer.pending):
                        target_onset_clock.reset()
                        target_timer.pending = True
                    elif num_targets == 0 and not in_passive_response:
                        response_on_clock.reset()
                        in_passive_response = True

            if left_fixation and not in_fixation:
                if not response_timer.on and in_trial and not in_feedback and not in_intertrial and not in_target:
                    # left fixation before response_timer came on
                    reward = False
                    failure = True
                    target_timer.pending = False
                    in_passive_response = False
                else:
                    reward = False
                    failure = False

            if target_timer.pending and (((num_targets > 0 and target_timer.idx == 0 and target_onset_clock.getTime() >= target_onset-frameinterval) or
            (0 < target_timer.idx < num_targets and inter_target_clock.getTime() >= inter_target_interval))):
                if target_timer.on == False and target_timer.off == True:
                    #target just came on
                    target_timer.pending = False
                    target.pos = stimulus_positions[target_timer.idx]
                    target_timer.pos_idx = params[all_stim_idx[combo_idx]][target_timer.idx][1]
                    target_timer.identity = stimulus_identity[target_timer.idx]
                    if target_timer.identity == 0:
                        target.color = red
                        target.opacity = 1.0
                    elif target_timer.identity == 1:
                        target.color = green
                        target.opacity = 1.0
                    else:
                        target.color = (0.0, 0.0, 0.0)  # blank target
                        target.opacity = 0.0
                    target_timer.update_marker()
                    if show_second_window:
                        target2.pos = target.pos
                        target2.color  = target.color
                    if response_cue_ref == 1:  # target onset
                        win.callOnFlip(target_timer.show, tracker, experiment_clock, timedict, response_on_clock)
                    else:  # target offset
                        win.callOnFlip(target_timer.show, tracker, experiment_clock,timedict)
                #target_on = True
            if target_timer.on and target_timer.off_clock.getTime() >= target_timer.duration - frameinterval:
                if target_timer.idx == num_targets-1: #we have seen all targets, prepare for response
                    if response_cue_ref == 0:
                        win.callOnFlip(target_timer.hide, tracker, experiment_clock,timedict, response_on_clock)
                    else:
                        win.callOnFlip(target_timer.hide, tracker, experiment_clock, timedict)
                elif target_timer.idx < num_targets-1: #still more targets to present
                    win.callOnFlip(target_timer.hide, tracker, experiment_clock,timedict,inter_target_clock)

            # response cue
            if (not (response_timer.on or in_feedback or in_intertrial or in_target)) and in_fixation and ((target_timer.idx >= num_targets and num_targets > 0) or (response_cue_ref == 1 and target_timer.on) or (response_cue_ref == 2 and response_on_clock.active)) and response_on_clock.finished(): #have we seen all targets
                if not response_timer.on:
                    if use_dummy:
                        print "Start of response period"
                    # reset the target position to the last (true) target
                    if num_targets > 0:
                        if 0 in stimulus_identity:
                           _idx = num_targets - (stimulus_identity[::-1].index(0)+1)
                        else:
                            _idx = -1  # This should never happen

                        target.pos = stimulus_positions[_idx]
                        target_window.pos = target.pos
                        if show_second_window:
                            target2.pos = target.pos
                            target2.color = red  # we are always going to target, i.e. red square
                            target_window2.pos = target2.pos
                    win.callOnFlip(response_timer.show, tracker,
                                    experiment_clock,
                                    timedict, in_response_clock)
                #in_response = True #response period just started
                    show_fixation_spot = False
            if make_response:
                if response_timer.on:
                    if target_window.contains(x,y):
                        if not in_target:
                            in_target_clock.reset()
                            target_fixation = (x, y)  # reset target fixation
                            n_target_fixation = 1
                            last_target_pos = (target.pos[0], target.pos[1])
                        else:
                            # update target fixation
                            target_fixation = (target_fixation[0]*n_target_fixation+x,
                                               target_fixation[1]*n_target_fixation+y)
                            n_target_fixation += 1
                            target_fixation = (target_fixation[0]/n_target_fixation,
                                               target_fixation[1]/n_target_fixation)

                            target_fixation = (x, y)  # reset target fixation
                        in_target = True
                        response_timer.on = False
                    elif in_response_clock.getTime() >= max_rtime + max_saccade_time:
                        failure = True
                        in_target = False
                        response_timer.on = False
                    elif in_response_clock.getTime() > max_rtime:
                        if in_fixation:
                            response_timer.on = False
                            in_target = False
                            failure = True  # the subject waited too long to respond
                    else:
                        response_timer.on = True
                    if show_second_window:
                        target_window2.draw()
                        target2.draw()

                if in_target:
                    if in_target_clock.getTime() >= target_fix_time:
                        reward = True
                        failure = False
                        in_target = False
                        if use_dummy:
                            print "Fulfilled target fixation criterion"
                    elif not target_window.contains(x,y):
                        in_target = False
                        failure = True
                        reward = False
                    else:
                        in_target = True
                        target_fixation = (x, y)  # update target fixation

            else:  # passive task
                if (not in_intertrial and not in_feedback and
                        response_timer.on and not (reward or failure)):
                    response_timer.on = False
                    if in_fixation:  # we have seen the response
                        if ((target_onset_ref == 1 and in_fixation_clock.finished())
                            or (target_onset_ref == 0 and target_timer.idx >= num_targets)):
                            in_fixation = False  # reset fixation tracking
                            reward = True
                            failure = False
                        in_passive_response = False
                    elif (not in_fixation) and left_fixation:
                        # we broke fixation before we were allowed to; failure
                        failure = True
                        reward = False
                        in_passive_response = False

            if (reward or failure) and not in_feedback:
                #reward of failure was set but we have not yet given feedback
                if reward:
                    if use_dummy:
                        timedict["reward_on"] = experiment_clock.getTime()
                        print "Reward given at %f" %(timedict["reward_on"],)
                    else:
                        send_message(tracker, "reward_on", experiment_clock, timedict)
                    if scale_reward:
                        pp = last_target_pos
                        _nn = failure_distr.get("nn", 0)
                        if _nn == 0:
                            pf = 0.0
                        else:
                            pf = 1.0*failure_distr.get(pp, 0)/_nn
                        ba = base_reward_duration
                        if pf > 1.0/num_target_locations:
                            ba = ba + (reward_duration - ba)*pf
                        dd = (abs(last_target_pos[0] - target_fixation[0]) +
                              abs(last_target_pos[1] - target_fixation[1]))
                        dd_max = target_window.size[0]/2 + target_window.size[1]/2
                        dd_min = target.size[0]/2 + target.size[1]/2
                        dd = 1-((min(dd_max, max(dd, dd_min)) - dd_min)/(dd_max - dd_min))
                        b = np.log(ba/reward_duration)
                        # Scale reward duration so that full reward is given when
                        # the subject fixates within the target
                        current_reward_duration = ba*np.exp(-b*dd**3)
                        # TODO: Add a scaling factor on the baseline (0.5) reward
                        #      duration so that rarely rewarded locations are rewarded
                        #      more.
                    else:
                        current_reward_duration = reward_duration
                    # update delay mean
                    delay_mean = ncorrect*delay_mean + response_cue_onset
                    delay_mean /= (ncorrect+1)
                    reward_cnx.open()  #start delivering reward
                    in_reward = True
                    reward_clock.reset()
                    if play_sound:
                        reward_sound.play()
                    reward = True
                    failure = False
                    ncorrect += 1
                    n_target_repeats = 0  # reset repeats
                elif failure:
                    timedict["failure"] = experiment_clock.getTime()
                    if use_dummy:
                        timedict["failure"] = experiment_clock.getTime()
                        print "Failure occurred at %f" %(timedict["failure"],)
                    else:
                        send_message(tracker, "failure", experiment_clock, timedict)
                    failure = True
                    reward = False
                    nfailures += 1
                    #repeat target if a failure was detected
                    if n_target_repeats < max_target_repeats:
                        if target_timer.active_this_trial:  # only increase this if we saw the target
                            n_target_repeats += 1
                    else:
                        n_target_repeats = 0

                    if target_timer.active_this_trial:  # only increase if we saw the target
                        # FIXME: This will not work if the failure e.g. after a
                        #        distractor following target, target.pos will then
                        #        represent the distractor position
                        pp = (target.pos[0], target.pos[1])
                        failure_distr[pp] = failure_distr.get(pp, 0) + 1
                        failure_distr["nn"] = failure_distr.get("nn", 0) + 1
                    target_timer.active = False #make sure we disable further targets
                if target_timer.on:
                    #make sure we turn off the target immediately upon feedback
                    win.callOnFlip(target_timer.hide, tracker, experiment_clock, timedict)

                in_feedback = True
                feedback_clock.reset()

            #check whether we are in the feedback phase of the trial
            if in_feedback and feedback_clock.getTime() < feedback_duration:
                if reward:
                    if reward_color is not None:
                        show_fixation_spot = True
                        fixation_spot.color = reward_color
                        if show_second_window:
                            fixation_spot2.setColor(reward_color,'rgb')
                elif failure:
                    if failure_color is not None:
                        show_fixation_spot = True
                        fixation_spot.setColor(failure_color,'rgb')
                        if show_second_window:
                            fixation_spot2.setColor(failure_color,'rgb')
                if in_reward:
                    rtime = reward_clock.getTime()
                    if rtime >= current_reward_duration:
                        in_reward = False
                        reward_total += rtime
                        reward_cnx.close()  #turn off the reward signal
                        if play_sound:
                            reward_sound.stop()
                        timedict["reward_off"] = experiment_clock.getTime()
                        if use_dummy:
                            timedict["reward_off"] = experiment_clock.getTime()
                            print "Reward off: %f" %(timedict["reward_off"],)
                        else:
                            send_message(tracker, "reward_off", experiment_clock, timedict)


                #make sure we switch off targets
            #check whether we have been in the feedback phase for the required time or we reached time out.
            elif ((in_feedback and feedback_clock.getTime() >= feedback_duration-frameinterval) or
                      (in_trial and (not in_feedback) and (trial_clock.getTime() >= trial_timeout-frameinterval) and not in_fixation)):
                #done with feedback; reset the fixation spot color
                if in_trial and not in_feedback:
                    nmissed += 1
                # Repeat the combo if the last trials was incorrect and we have
                # not yet exceeded the maximum number of repetitions
                if n_target_repeats == 0:
                    combo_idx += 1  # update the combo idx after the trial has finished
                in_trial = False
                reward = False
                failure = False
                in_feedback = False
                left_fixation = False
                in_passive_response = False
                fixation_spot.setColor(white,'rgb')
                if show_second_window:
                    fixation_spot2.setColor(white,'rgb')
                #make sure we close the reward
                if timedict["reward_on"] > 0.0 and timedict["reward_off"] == 0.0:
                    #need to switch off the reward
                    rtime = reward_clock.getTime()
                    reward_cnx.close()
                    send_message(tracker,"reward_off",experiment_clock,timedict)
                    reward_total += rtime
                in_reward = False
                response_on_clock.active = False  # make sure we deactivate response clock
                #start the ITI clock
                intertrial_clock.reset()
                in_intertrial = True
                if use_dummy:
                    timedict["trial_end"] = experiment_clock.getTime()
                    print "Trial end at %f" %(timedict["trial_end"],)
                else:
                    send_message(tracker, "trial_end", experiment_clock, timedict)
                if autocycle:
                    fixation_pos = fixation_locations[str(np.random.randint(npositions))]
                if show_second_window:
                    fixation_mean_spot.setPos(fix_distr[tuple(fixation_pos)][:2])

                target_timer.active_this_trial = False  # reset target activity tracker
                # save data for this trial here
                trial_start_times.append(timedict["trial_start"])
                trial_end_times.append(timedict["trial_end"])
                fix_start_times.append(timedict["fix_start"])
                reward_times.append(timedict["reward_on"])
                reward_off_times.append(timedict["reward_off"])
                failure_times.append(timedict["failure"])
                target_on_times.append(timedict["target_on"])
                target_off_times.append(timedict["target_off"])
                response_on_times.append(timedict["response_on"])
                left_fixation_times.append(timedict["left_fixation"])
                target_positions.append(stimulus_positions)
                stimulus_identities.append(stimulus_identity)
                fixation_positions.append(fixation_spot.pos)
                # reset times
                for k in timedict.keys():
                    timedict[k] = 0.0
                # write trial progress to stoout
                sys.stdout.write("\rTrials: correct: %d (%.1f%%); failed: %d (%.1f%%); missed %d (%.1f%%); total: %d Reward: %.2f ml Mean delay: %.2f s\n" % (
                                 ncorrect, 100*ncorrect/ntrials, nfailures, 100*nfailures/ntrials,
                                 nmissed, 100*nmissed/ntrials, ntrials, reward_total*droprate, delay_mean))

            if in_intertrial and intertrial_clock.getTime() < intertrial_duration:
                show_fixation_spot = False
                # update fixation position
                fixation_spot.setPos(fixation_pos)
                fixation_window.setPos(fixation_pos)
                if show_second_window:
                    fixation_spot2.setPos(fixation_pos)
                    fixation_window2.setPos(fixation_pos)

            if in_intertrial and intertrial_clock.getTime() >= intertrial_duration:
               in_intertrial = False
               if in_trial == False:
                   trial_clock.reset()
                   if response_cue_onset_min == response_cue_onset_max:
                       response_cue_onset = response_cue_onset_min
                   else:
                       response_cue_onset = (response_cue_onset_min +
                                             (response_cue_onset_max - response_cue_onset_min)*np.random.random())
                   response_on_clock.duration = response_cue_onset
                   ntrials += 1
                   target_timer.idx = 0
                   target_timer.turn_off()
                   if combo_idx == len(all_stim_idx):
                       # Shuffle for the next block
                       np.random.shuffle(all_stim_idx)
                       combo_idx = 0
                   stimulus_identity = [params[all_stim_idx[combo_idx]][i][0] for i in xrange(num_targets)]
                   stimulus_positions = [target_locations[params[all_stim_idx[combo_idx]][i][1]] for i in xrange(num_targets)]
                   if len(stimulus_positions) > 0:
                       target.setPos(stimulus_positions[0])
                   target_timer.active = True #re-activate targets
                   if target_onset_ref == 1:  #show target at the start of the trial
                       target_onset_clock.reset()
                       target_timer.pending = True
                   else:
                       target_timer.pending = False
                   if show_second_window:
                       target2.setPos(target.pos)
                   if use_dummy:
                       timedict["trial_start"] = experiment_clock.getTime()
                       print "Trial start at %f" %(timedict["trial_start"], )
                   else:
                       send_message(tracker, "trial_start", experiment_clock, timedict)
                   in_trial = True
               show_fixation_spot = True
               fixation_spot.setColor(white, 'rgb')
               if show_second_window:
                   fixation_spot2.setColor(white,'rgb')

            if show_fixation_spot:
                fixation_spot.draw()
                if show_second_window:
                    fixation_window2.draw()
                    fixation_spot2.draw()

            if draw_eyespot:
                #eye_spot.draw()
                if show_second_window:
                    eye_spot2.draw()
                    fixation_mean_spot.draw()
            if show_anchors:
                anchors.draw()

            if target_timer.on:
                target.draw()
                if show_second_window:
                    target2.draw()
            #timing
            tt = step_clock.getTime()
            t_mean  += tt
            t_std  += tt*tt
            #display everything
            win.flip()
            if show_second_window:
                win2.flip()
            nframes += 1
    except:
        tb.print_exc(file=sys.stdout)
    finally:

        win.close()
        if show_second_window:
            win2.close()
        if not use_dummy:
            tracker.record_off()
            tracker.end_experiment(os.getcwd())
        if not reward_cnx.dummy:
            reward_cnx.port.close()
        t_mean /= nframes
        t_std /= nframes
        total_time = cresults.get("total_time", 0.0) + experiment_clock.getTime()
        print "Step time: %.2f +/- %.2f ms" %(1000*t_mean, 1000*np.sqrt(t_std - t_mean*t_mean))
        if cresults:
            if not block_report:
                block_ntrials = ntrials - cresults.get("ntrials", 0)
                block_ncorrect = ncorrect - cresults.get("ncorrect", 0)
                block_nfailures = nfailures - cresults.get("nfailures", 0)
                block_nmissed = nmissed - cresults.get("nmissed", 0)
                block_reward = reward_total - cresults.get("reward_total", 0.0)
                block_manual_reward = manual_reward_total - cresults.get("manual_reward_total", 0.0)
            else:
                block_ntrials = ntrials
                block_ncorrect = ncorrect
                block_nfailures = nfailures
                block_nmissed = nmissed
                block_manual_reward = manual_reward_total
                block_reward = reward

            print "This block:"
            print "\tNumber of trials: %d; Number of correct: %d; Number of failures: %d; Number of missed: %d" %(block_ntrials,
                                                                                                                block_ncorrect,
                                                                                                            block_nfailures,
                                                                                                                block_nmissed)
        else:
            block_ntrials = ntrials
            block_ncorrect = ncorrect
            block_nfailures = nfailures
            block_nmisssed = nmissed
            block_reward = reward_total
            block_manual_reward = manual_reward_total

        if block_report:
            #need to add previous trials
            ntrials += cresults.get("ntrials", 0)
            ncorrect += cresults.get("ncorrect", 0)
            nfailures += cresults.get("nfailures", 0)
            nmissed += cresults.get("nmissed", 0)
        print "Total:"
        print "\tNumber of trials: %d; Number of correct: %d; Number of failures: %d; Number of missed: %d" %(ntrials,
                                                                                                    ncorrect,
                                                                                                    nfailures,
                                                                                                    nmissed)
        print "Total reward given: %f" %(reward_total*droprate,)
        print "Total manual reward given: %f" %(manual_reward_total*droprate,)
        print "Experimental time: %f" %(total_time, )
        print "Frame interval: %f" % (frameinterval,)
        fixation_x_positions = [x[0] for x in fixation_positions]
        fixation_y_positions = [x[1] for x in fixation_positions]
        rr = np.rec.fromarrays((trial_start_times, fix_start_times, reward_times, reward_off_times, failure_times, trial_end_times,
                                target_on_times, target_off_times,response_on_times, left_fixation_times,
                                fixation_x_positions, fixation_y_positions),
                               dtype=[("trial_start", np.float), ("fix_start", np.float), ("reward_on", np.float),
                                      ("reward_off", np.float), ("failure", np.float), ("trial_end", np.float),
                                      ("target_on", np.float), ("target_off",np.float),("response_on", np.float),
                                      ("left_fixation", np.float),("fixation_x", np.float), ("fixation_y", np.float)])

        if num_targets > 0:
            for i in xrange(num_targets):
                target_x_positions = [x[i][0] for x in target_positions]
                target_y_positions = [x[i][1] for x in target_positions]
                rr = np.lib.recfunctions.append_fields(rr, ("target_%d_x" %(i,),"target_%d_y" %(i,)),
                                                      data = [target_x_positions,target_y_positions],
                                                       usemask=False)
            for i in xrange(num_targets):
                _stim_id = [x[i] for x in stimulus_identities]
                rr = np.lib.recfunctions.append_fields(rr, "stim_%d_id" %(i,),_stim_id,usemask=False)

        mlab.rec2csv(rr, filename + "_results.txt")
        manual_reward_duration_total = sum(map(lambda x,y : y-x, manual_reward_on_times, manual_reward_off_times))
        summary_plot(rr, manual_reward_duration_total, exp_info.get("droprate", 1.0),
                     filename=filename + "_summary.pdf")
        results = {"reward_total": reward_total,
                   "manual_reward_total": manual_reward_total,
                   "ntrials": ntrials,
                   "ncorrect": ncorrect,
                   "nfailures": nfailures,
                   "nmissed": nmissed,
                   "total_time": total_time,
                   "fix_distr": fix_distr}

    return results,""


if __name__ == "__main__":
    run_experiment({"use_dummy":True,
                    "screen_width":1280,
                    "screen_height":800,
                    "fixation_size":3.0,
                    "mask":"gauss"})
