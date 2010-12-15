#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('wbc_pr2_ctrl')

import rospy
from wbc_pr2_ctrl import srv

import Tix
import math
from functools import partial


class Slider(Tix.Frame):
    def __init__(self, parent, name, unit, lower, upper, value):
        Tix.Frame.__init__(self, parent)
        self.unit_scale = 1
        if unit == 'rad' or unit == 'radians':
            unit = 'deg'
            self.unit_scale = math.pi / 180
        elif unit == 'm' or unit == 'meters':
            unit = 'mm'
            self.unit_scale = 0.001;
        lower /= self.unit_scale
        upper /= self.unit_scale
        value /= self.unit_scale
        Tix.Label(self, text = '%s [%s]  (%5.2f to %5.2f)' % (name, unit, lower, upper), anchor = 'w')\
            .pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.value_var = Tix.DoubleVar(self)
        self.value_var.set(value)
        frame = Tix.Frame(self)
        Tix.Label(frame, text = 'value', width = 6).pack(side = Tix.LEFT)
        self.value_scale = Tix.Scale(frame, orient = Tix.HORIZONTAL,
                                     variable = self.value_var, showvalue = 0,
                                     from_ = lower, to = upper,
                                     command = self._UpdateValueLabel)
        self.value_scale.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        self.FORMAT = '%5.2f'
        self.value_label = Tix.Label(frame, text = self.FORMAT % value, width = 12)
        self.value_label.pack(side = Tix.LEFT)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.lower = lower
        self.upper = upper
        self.master_var = None

    def _ConvertMaster(self, value):
        return self.lower + (self.upper - self.lower) * value / 100.0

    def _UpdateValueLabel(self, value):
        self.value_label['text'] = self.FORMAT % float(value)

    def _UpdateMaster(self, value):
        value = self._ConvertMaster(value)
        self.value_var.set(value)
        self._UpdateValueLabel(value)

    def GetValue(self):
        if self.master_var:
            return self.unit_scale * self._ConvertMaster(self.master_var.get())
        return self.unit_scale * self.value_var.get()
    
    def Override(self, master_var):
        self.value_scale['state'] = Tix.DISABLED
        self.value_scale['sliderrelief'] = Tix.FLAT
        self.value_scale['sliderlength'] = 3
        self.master_var = master_var
        self._UpdateMaster(master_var.get())
        
    def Release(self):
        self.value_scale['state'] = Tix.NORMAL
        self.value_scale['sliderrelief'] = Tix.RAISED
        self.value_scale['sliderlength'] = 30
        if self.master_var:
            self.value_var.set(self.lower + (self.upper - self.lower) * self.master_var.get() / 100.0)
            self.master_var = None
        self.value_label['text'] = self.FORMAT % float(self.value_var.get())
        
        
class SliderStack(Tix.Frame):
    def __init__(self, parent):
        Tix.Frame.__init__(self, parent)
        self.slider = []
        self.master_override = Tix.IntVar(self)
        self.master_override.set(0)
        frame = Tix.Frame(self)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.master_check = Tix.Checkbutton(frame, text = 'override', variable = self.master_override,
                                            command = self.Toggle)
        self.master_check.pack(side = Tix.LEFT)
        self.master_var = Tix.DoubleVar(self)
        self.master_var.set(0)
        self.master_scale = Tix.Scale(frame, orient = Tix.HORIZONTAL,
                                      variable = self.master_var, showvalue = 0,
                                      from_ = 0, to = 100, command = self._UpdateMaster)
        self.master_scale.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        Tix.Label(frame, textvariable = self.master_var, width = 12).pack(side = Tix.LEFT)
        self.gframe = Tix.Frame(self)
        self.gframe.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        
    def _UpdateMaster(self, value):
        if self.master_override.get() == 1:
            value = float(value)
            for ii in self.slider:
                ii._UpdateMaster(value)
        
    def Toggle(self):
        if self.master_override.get() == 1:
            for ii in self.slider:
                ii.Override(self.master_var)
        else:
            for ii in self.slider:
                ii.Release()

    def Reset(self, names, units, lower_bounds, upper_bounds, values):
        '''
        We need at least names and values to be arrays of the same
        dimension. If the units and lower or upper bounds are empty,
        they will simple be replaced by some default. If you do
        specify them, their dimension must match the names and values.
        '''
        dim = len(names)
        if 0 == dim:
            raise RuntimeError('names must not be empty')
        if len(values) != dim:
            raise RuntimeError('values (length %d) must have same length as names (length %d)' % (len(values), dim))
        if 0 == len(units):
            units = ['none'] * dim
        if 0 == len(lower_bounds):
            lower_bounds = [-1.0] * dim
        if 0 == len(upper_bounds):
            upper_bounds = [1.0] * dim
        if len(units) != dim:
            raise RuntimeError('units (length %d) must have same length as names (length %d)' % (len(units), dim))
        if len(lower_bounds) != dim:
            raise RuntimeError('lower_bounds (length %d) must have same length as names (length %d)' % (len(lower_bounds), dim))
        if len(upper_bounds) != dim:
            raise RuntimeError('upper_bounds (length %d) must have same length as names (length %d)' % (len(upper_bounds), dim))
        
        for ii in self.slider:
            ii.destroy()
        self.slider = []
        self.master_override.set(0)
        self.master_var.set(0)
        for ii in xrange(len(values)):
            # heuristic for handling continuous joints etc: assume
            # they are revolute, and set limits such that we get two
            # full revolutions on the goal slider.
            ll = lower_bounds[ii]
            uu = upper_bounds[ii]
            if ll < -10000.0:
                if uu > 10000.0:
                    ll = -2.0 * math.pi
                else:
                    ll = -4.0 * math.pi
            if uu > 10000.0:
                if ll < -10000.0:
                    uu = 2.0 * math.pi
                else:
                    uu = 4.0 * math.pi
            newslider = Slider(self.gframe, names[ii], units[ii], ll, uu, values[ii])
            newslider.pack(side = Tix.TOP, expand = True, fill = Tix.X)
            self.slider.append(newslider)
            
    def Read(self):
        data = list()
        for ii in self.slider:
            data.append(ii.GetValue())
        return data


class ValueSetter(Tix.Frame):
    def __init__(self, parent, mode_name, mode_id):
        Tix.Frame.__init__(self, parent, bd = 2, relief = Tix.RIDGE)
        frame = Tix.Frame(self)
        self.mode_name = mode_name
        self.mode_id = mode_id
        self.button = Tix.Button(frame, text = '%s (%d)' % (mode_name, mode_id), command = self.Set)
        self.button.pack(side = Tix.LEFT)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(no status yet)')
        self.label = Tix.Label(frame, textvariable = self.labeltext)
        self.label.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.sliders = SliderStack(self)
        self.sliders.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.task_posture_ui_srv = None

    def Reset(self, names, units, lower_bounds, upper_bounds, values):
        self.sliders.Reset(names, units, lower_bounds, upper_bounds, values)
        
    def LazyInit(self):
        if not self.task_posture_ui_srv:
            self.labeltext.set('lazy init...')
            rospy.wait_for_service('/wbc_pr2_ctrl_wbc_plugin/ui')
            self.task_posture_ui_srv = rospy.ServiceProxy('/wbc_pr2_ctrl_wbc_plugin/ui', srv.TaskPostureUI)
            self.labeltext.set('lazy init DONE')
        
    def Set(self):
        try:
            self.LazyInit()
            value = self.sliders.Read()
            response = self.task_posture_ui_srv(self.mode_id, value)
            if not response.ok:
                self.labeltext.set('%s' % response.errstr)
            else:
                self.labeltext.set('ok')
        except rospy.ServiceException, ee:
            self.labeltext.set('EXCEPTION: %s' % ee)


class Initializer(Tix.Frame):
    def __init__(self, parent, task_goal, task_kp, task_kd, posture_goal, posture_kp, posture_kd):
        Tix.Frame.__init__(self, parent)
        self.task_goal = task_goal
        self.task_kp = task_kp
        self.task_kd = task_kd
        self.posture_goal = posture_goal
        self.posture_kp = posture_kp
        self.posture_kd = posture_kd
        self.button = Tix.Button(self, text = 'Initialize', bg = '#66cc66', command = self.Initialize)
        self.button.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(not initialized yet)')
        self.label = Tix.Label(self, textvariable = self.labeltext, anchor = 'w')
        self.label.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.task_posture_ui_srv = None
        
    def LazyInit(self):
        if not self.task_posture_ui_srv:
            self.labeltext.set('lazy init...')
            rospy.wait_for_service('/wbc_pr2_ctrl_wbc_plugin/ui')
            self.task_posture_ui_srv = rospy.ServiceProxy('/wbc_pr2_ctrl_wbc_plugin/ui', srv.TaskPostureUI)
            self.labeltext.set('lazy init DONE')

    def Initialize(self):
        try:
            self.LazyInit()

            resp = self.task_posture_ui_srv(srv.TaskPostureUIRequest.GET_TASK_GOAL, [])
            if not resp.ok:
                raise 'GET_TASK_GOAL error: %s' % resp.errstr
            self.task_goal.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)

            resp = self.task_posture_ui_srv(srv.TaskPostureUIRequest.GET_TASK_KP, [])
            if not resp.ok:
                raise 'GET_TASK_KP error: %s' % resp.errstr
            self.task_kp.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)

            resp = self.task_posture_ui_srv(srv.TaskPostureUIRequest.GET_TASK_KD, [])
            if not resp.ok:
                raise 'GET_TASK_KD error: %s' % resp.errstr
            self.task_kd.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)

            resp = self.task_posture_ui_srv(srv.TaskPostureUIRequest.GET_POSTURE_GOAL, [])
            if not resp.ok:
                raise 'GET_POSTURE_GOAL error: %s' % resp.errstr
            self.posture_goal.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)

            resp = self.task_posture_ui_srv(srv.TaskPostureUIRequest.GET_POSTURE_KP, [])
            if not resp.ok:
                raise 'GET_POSTURE_KP error: %s' % resp.errstr
            self.posture_kp.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)

            resp = self.task_posture_ui_srv(srv.TaskPostureUIRequest.GET_POSTURE_KD, [])
            if not resp.ok:
                raise 'GET_POSTURE_KD error: %s' % resp.errstr
            self.posture_kd.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)

            self.labeltext.set('initialized')

        except rospy.ServiceException, ee:
            self.labeltext.set('EXCEPTION: %s' % ee)


if __name__ == '__main__':
    root = Tix.Tk()
    root.title('TaskPosture GUI')
    
    top_frame = Tix.Frame(root, bd = 2, relief = Tix.RIDGE)
    middle_frame = Tix.Frame(root, bd = 2, relief = Tix.RIDGE)
    bottom_frame = Tix.Frame(root, bd = 2, relief = Tix.RIDGE)

    task_goal    = ValueSetter(middle_frame, "SET_TASK_GOAL",    srv.TaskPostureUIRequest.SET_TASK_GOAL)
    task_kp      = ValueSetter(middle_frame, "SET_TASK_KP",      srv.TaskPostureUIRequest.SET_TASK_KP)
    task_kd      = ValueSetter(middle_frame, "SET_TASK_KD",      srv.TaskPostureUIRequest.SET_TASK_KD)
    
    posture_goal = ValueSetter(bottom_frame, "SET_POSTURE_GOAL", srv.TaskPostureUIRequest.SET_POSTURE_GOAL)
    posture_kp   = ValueSetter(bottom_frame, "SET_POSTURE_KP",   srv.TaskPostureUIRequest.SET_POSTURE_KP)
    posture_kd   = ValueSetter(bottom_frame, "SET_POSTURE_KD",   srv.TaskPostureUIRequest.SET_POSTURE_KD)
    
    quitter = Tix.Button(top_frame, text = 'QUIT', command = root.quit, bg = '#cc6666')
    initializer = Initializer(top_frame, task_goal, task_kp, task_kd, posture_goal, posture_kp, posture_kd)
    
    top_frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
    middle_frame.pack(side = Tix.TOP, expand = True, fill = Tix.BOTH)
    bottom_frame.pack(side = Tix.TOP, expand = True, fill = Tix.BOTH)
    
    task_goal.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    task_kp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    task_kd.pack(side = Tix.LEFT, expand = True, fill = Tix.X)

    posture_goal.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    posture_kp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    posture_kd.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    
    initializer.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    quitter.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    
    root.mainloop()
