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


class ValueProxy(Tix.Frame):
    def __init__(self, parent, name, get_mode, set_mode):
        Tix.Frame.__init__(self, parent, bd = 2, relief = Tix.RIDGE)
        self.name = name
        self.get_mode = get_mode
        self.set_mode = set_mode
        self.set_btn = Tix.Button(self, text = 'Set %s (%d)' % (name, set_mode), command = self.Set)
        self.set_btn.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.get_btn = Tix.Button(self, text = 'Get %s (%d)' % (name, get_mode), command = self.Get)
        self.get_btn.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(no status yet)')
        self.label = Tix.Label(self, textvariable = self.labeltext)
        self.label.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.sliders = SliderStack(self)
        self.sliders.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.task_posture_ui_srv = None
            
    def LazyInit(self):
        if not self.task_posture_ui_srv:
            self.labeltext.set('lazy init...')
            service_name = '/wbc_pr2_ctrl/tp_ui'
            rospy.wait_for_service(service_name)
            self.task_posture_ui_srv = rospy.ServiceProxy(service_name, srv.TaskPostureUI)
            self.labeltext.set('lazy init DONE')

    def Get(self):
        try:
            self.LazyInit()
            resp = self.task_posture_ui_srv(self.get_mode, [])
            if not resp.ok:
                raise 'Get %s error: %s' % (self.name, resp.errstr)
            self.sliders.Reset(resp.name, resp.unit, resp.lower_bound, resp.upper_bound, resp.value)
        except rospy.ServiceException, ee:
            self.labeltext.set('Get %s exception: %s' % (self.name, ee))
        
    def Set(self):
        try:
            self.LazyInit()
            value = self.sliders.Read()
            response = self.task_posture_ui_srv(self.set_mode, value)
            if not response.ok:
                self.labeltext.set('%s' % response.errstr)
            else:
                self.labeltext.set('ok')
        except rospy.ServiceException, ee:
            self.labeltext.set('Set %s exception: %s' % (self.name, ee))


class Initializer(Tix.Frame):
    def __init__(self, parent, proxies):
        Tix.Frame.__init__(self, parent)
        self.proxies = proxies
        self.button = Tix.Button(self, text = 'Initialize', bg = '#66cc66', command = self.Initialize)
        self.button.pack(side = Tix.TOP, expand = True, fill = Tix.X)

    def Initialize(self):
        for pp in proxies:
            pp.Get()


if __name__ == '__main__':
    root = Tix.Tk()
    root.title('TaskPosture GUI')
    
    frames = []
    for ii in xrange(3):
        ff = Tix.Frame(root, bd = 2, relief = Tix.RIDGE)
        ff.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        frames.append(ff)
    
    proxies = []
    pp = ValueProxy(frames[1], "CONTROL_POINT",
                    srv.TaskPostureUIRequest.GET_CONTROL_POINT,
                    srv.TaskPostureUIRequest.SET_CONTROL_POINT)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    pp = ValueProxy(frames[1], "TASK_GOAL",
                    srv.TaskPostureUIRequest.GET_TASK_GOAL,
                    srv.TaskPostureUIRequest.SET_TASK_GOAL)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    pp = ValueProxy(frames[1], "TASK_KP",
                    srv.TaskPostureUIRequest.GET_TASK_KP,
                    srv.TaskPostureUIRequest.SET_TASK_KP)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    pp = ValueProxy(frames[1], "TASK_KD",
                    srv.TaskPostureUIRequest.GET_TASK_KD,
                    srv.TaskPostureUIRequest.SET_TASK_KD)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    pp = ValueProxy(frames[2], "POSTURE_GOAL",
                    srv.TaskPostureUIRequest.GET_POSTURE_GOAL,
                    srv.TaskPostureUIRequest.SET_POSTURE_GOAL)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    pp = ValueProxy(frames[2], "POSTURE_KP",
                    srv.TaskPostureUIRequest.GET_POSTURE_KP,
                    srv.TaskPostureUIRequest.SET_POSTURE_KP)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    pp = ValueProxy(frames[2], "POSTURE_KD",
                    srv.TaskPostureUIRequest.GET_POSTURE_KD,
                    srv.TaskPostureUIRequest.SET_POSTURE_KD)
    pp.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    proxies.append(pp)
    
    initializer = Initializer(frames[0], proxies)
    initializer.pack(side = Tix.LEFT, expand = True, fill = Tix.X)

    quitter = Tix.Button(frames[0], text = 'QUIT', command = root.quit, bg = '#cc6666')
    quitter.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        
    root.mainloop()
