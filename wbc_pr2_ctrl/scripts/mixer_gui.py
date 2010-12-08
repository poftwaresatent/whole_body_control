#!/usr/bin/env python

import sys
import roslib
roslib.load_manifest('pr2_stanford_wbc')

import rospy
from pr2_stanford_wbc import srv
from pr2_stanford_wbc import msg

import Tix
import math
from functools import partial


class GoalSlider(Tix.Frame):
    def __init__(self, parent, name, unit, lower, upper, goal, actual):
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
        goal /= self.unit_scale
        actual /= self.unit_scale
        Tix.Label(self, text = '%s [%s]  (%5.2f to %5.2f)' % (name, unit, lower, upper), anchor = 'w')\
            .pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.goal_var = Tix.DoubleVar(self)
        self.goal_var.set(goal)
        frame = Tix.Frame(self)
        Tix.Label(frame, text = 'goal', width = 6).pack(side = Tix.LEFT)
        self.goal_scale = Tix.Scale(frame, orient = Tix.HORIZONTAL,
                                    variable = self.goal_var, showvalue = 0,
                                    from_ = lower, to = upper,
                                    command = self._UpdateGoalLabel)
        self.goal_scale.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        self.FORMAT = '%5.2f'
        self.goal_label = Tix.Label(frame, text = self.FORMAT % goal, width = 12)
        self.goal_label.pack(side = Tix.LEFT)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.actual_var = Tix.DoubleVar(self)
        self.actual_var.set(actual)
        frame = Tix.Frame(self)
        Tix.Label(self, text = 'actual', width = 6).pack(side = Tix.LEFT)
        self.actual_scale = Tix.Scale(self, orient = Tix.HORIZONTAL,
                                      state = Tix.DISABLED, sliderrelief = Tix.FLAT, sliderlength = 3,
                                      variable = self.actual_var, showvalue = 0,
                                      from_ = lower, to = upper,
                                      command = self._UpdateGoalLabel)
        self.actual_scale.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        self.actual_label = Tix.Label(frame, text = self.FORMAT % actual, width = 12)
        self.actual_label.pack(side = Tix.LEFT)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.lower = lower
        self.upper = upper
        self.master_var = None

    def _ConvertMaster(self, value):
        return self.lower + (self.upper - self.lower) * value / 100.0

    def _UpdateGoalLabel(self, goal):
        self.goal_label['text'] = self.FORMAT % float(goal)

    def _UpdateActualLabel(self, actual):
        self.actual_label['text'] = self.FORMAT % float(actual)
        
    def _UpdateMaster(self, value):
        goal = self._ConvertMaster(value)
        self.goal_var.set(goal)
        self._UpdateGoalLabel(goal)

    def GetGoal(self):
        if self.master_var:
            return self.unit_scale * self._ConvertMaster(self.master_var.get())
        return self.unit_scale * self.goal_var.get()
    
    def Override(self, master_var):
        self.goal_scale['state'] = Tix.DISABLED
        self.goal_scale['sliderrelief'] = Tix.FLAT
        self.goal_scale['sliderlength'] = 3
        self.master_var = master_var
        self._UpdateMaster(master_var.get())
        
    def Release(self):
        self.goal_scale['state'] = Tix.NORMAL
        self.goal_scale['sliderrelief'] = Tix.RAISED
        self.goal_scale['sliderlength'] = 30
        if self.master_var:
            self.goal_var.set(self.lower + (self.upper - self.lower) * self.master_var.get() / 100.0)
            self.master_var = None
        self.goal_label['text'] = self.FORMAT % float(self.goal_var.get())


class BunchOfGoalSliders(Tix.Frame):
    def __init__(self, parent):
        Tix.Frame.__init__(self, parent)
        self.goal_slider = []
        self.master_override = Tix.IntVar(self)
        self.master_override.set(0)
        frame = Tix.Frame(self)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.master_check = Tix.Checkbutton(frame, text = 'master', variable = self.master_override,
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
            for ii in self.goal_slider:
                ii._UpdateMaster(value)
        
    def Toggle(self):
        if self.master_override.get() == 1:
            for ii in self.goal_slider:
                ii.Override(self.master_var)
        else:
            for ii in self.goal_slider:
                ii.Release()

    def Reset(self, names, units, goals, actuals, lower, upper):
        if (len(names) != len(goals)) \
                or (len(names) != len(actuals)) \
                or (len(names) != len(lower)) \
                or (len(names) != len(upper)):
            raise RuntimeError('you gave me %d names, %d goals, %d actuals, %d lower limits, and %d upper limits' % (len(names), len(goals), len(actuals), len(lower), len(upper)))
        for ii in self.goal_slider:
            ii.destroy()
        self.goal_slider = []
        self.master_override.set(0)
        self.master_var.set(0)
        for ii in xrange(len(goals)):
            # heuristic for handling continuous joints etc: assume
            # they are revolute, and set limits such that we get two
            # full revolutions on the goal slider.
            ll = lower[ii]
            uu = upper[ii]
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
            gs = GoalSlider(self.gframe, names[ii], units[ii], ll, uu, goals[ii], actuals[ii])
            gs.pack(side = Tix.TOP, expand = True, fill = Tix.X)
            self.goal_slider.append(gs)
            
    def Read(self):
        data = list()
        for ii in self.goal_slider:
            data.append(ii.GetGoal())
        return data


class GoalSetter(Tix.Frame):
    def __init__(self, parent):
        Tix.Frame.__init__(self, parent, bd = 2, relief = Tix.RIDGE)
        frame = Tix.Frame(self)
        self.button = Tix.Button(frame, text = 'SetGoal', command = self.SetGoal)
        self.button.pack(side = Tix.LEFT)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(no status yet)')
        self.label = Tix.Label(frame, textvariable = self.labeltext)
        self.label.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.goal_sliders = BunchOfGoalSliders(self)
        self.goal_sliders.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.set_goal = None

    def Reset(self, names, units, goal, actual, limit_lower, limit_upper):
        self.goal_sliders.Reset(names, units, goal, actual, limit_lower, limit_upper)
        
    def LazyInit(self):
        if not self.set_goal:
            self.labeltext.set('lazy init...')
            rospy.wait_for_service('/jspace_servo/set_goal')
            self.set_goal = rospy.ServiceProxy('/jspace_servo/set_goal', srv.SetGoal)
            self.labeltext.set('lazy init DONE')
        
    def SetGoal(self):
        try:
            self.LazyInit()
            goal = self.goal_sliders.Read()
            response = self.set_goal(goal)
            if not response.ok:
                self.labeltext.set('%s' % response.errstr)
            else:
                self.labeltext.set('ok')
        except rospy.ServiceException, ee:
            self.labeltext.set('EXCEPTION: %s' % ee)


class GainSlider(Tix.Frame):
    def __init__(self, parent, prefixes, name, maxvals, gains):
        if (len(prefixes) != len(maxvals)) or (len(prefixes) != len(gains)):
            raise 'you gave me %d prefixes, %d maxvals, and %d gains' % (len(prefixes), len(maxvals), len(gains))
        Tix.Frame.__init__(self, parent)
        self.name = Tix.Label(self, text = 'name: %s' % name, anchor = 'w')
        self.name.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.gain_var = []
        self.gain_label = []
        self.gain_scale = []
        self.master_var = []
        for ii in xrange(len(prefixes)):
            gv = Tix.DoubleVar(self)
            gv.set(gains[ii])
            frame = Tix.Frame(self)
            Tix.Label(frame, text = prefixes[ii], width = 6).pack(side = Tix.LEFT)
            gs = Tix.Scale(frame, orient = Tix.HORIZONTAL,
                           variable = gv, showvalue = 0,
                           from_ = 0, to = maxvals[ii])
            gs.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
            gl = Tix.Label(frame, textvariable = gv, width = 12)
            gl.pack(side = Tix.LEFT)
            frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
            self.gain_var.append(gv)
            self.gain_label.append(gl)
            self.gain_scale.append(gs)
            self.master_var.append(None)

    def SetGains(self, gains):
        if len(gains) != len(self.gain_var):
            raise 'you gave me %d gains but I want %d' % (len(gains), len(self.gain_var))
        for ii in xrange(len(gains)):
            self.gain_var[ii].set(gain[ii])

    def GetGain(self, index):
        if (index < 0) or (index >= len(self.gain_var)):
            raise 'oops, index is %d but len(self.gain_var) is %d' % (index, len(self.gain_var))
        if self.master_var[index]:
            return self.master_var[index].get()
        return self.gain_var[index].get()
    
    def Override(self, index, master_var):
        if (index < 0) or (index >= len(self.gain_scale)):
            raise 'index %d is out of range (I only have %d gains)' % (index, len(self.gain_scale))
        self.gain_label[index]['textvariable'] = master_var
        self.gain_scale[index]['state'] = Tix.DISABLED
        self.gain_scale[index]['variable'] = master_var
        self.master_var[index] = master_var
        
    def Release(self, index):
        if (index < 0) or (index >= len(self.gain_scale)):
            raise 'index %d is out of range (I only have %d gains)' % (index, len(self.gain_scale))
        self.gain_label[index]['textvariable'] = self.gain_var[index]
        self.gain_scale[index]['state'] = Tix.NORMAL
        self.gain_scale[index]['variable'] = self.gain_var[index]
        if self.master_var[index]:
            self.gain_var[index].set(self.master_var[index].get())
            self.master_var[index] = None


class BunchOfGainSliders(Tix.Frame):
    def __init__(self, parent, prefixes, maxvals):
        if len(prefixes) != len(maxvals):
            raise 'you gave me %d prefixes but %d maxvals' % (len(prefixes), len(maxvals))
        Tix.Frame.__init__(self, parent)
        self.gain_slider = []
        self.prefixes = prefixes
        self.maxvals = maxvals
        self.master_override = []
        self.master_check = []
        self.master_var = []
        self.master_scale = []
        for ii in xrange(len(prefixes)):
            mo = Tix.IntVar(self)
            mo.set(0)
            frame = Tix.Frame(self)
            frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
            mc = Tix.Checkbutton(frame, text = 'master %s' % prefixes[ii], variable = mo,
                                 command = partial(self.Toggle, ii))
            mc.pack(side = Tix.LEFT)
            mv = Tix.DoubleVar(self)
            mv.set(0)
            ms = Tix.Scale(frame, orient = Tix.HORIZONTAL, variable = mv, showvalue = 0, from_ = 0, to = maxvals[ii])
            ms.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
            Tix.Label(frame, textvariable = mv, width = 12).pack(side = Tix.LEFT)
            self.master_override.append(mo)
            self.master_check.append(mc)
            self.master_var.append(mv)
            self.master_scale.append(ms)
        self.gframe = Tix.Frame(self)
        self.gframe.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        
    def Toggle(self, index):
        if (index < 0) or (index >= len(self.master_var)):
            raise 'oops, index is %d but len(self.master_var) is %d' % (index, len(self.master_var))
        if self.master_override[index].get() == 1:
            for ii in self.gain_slider:
                ii.Override(index, self.master_var[index])
        else:
            for ii in self.gain_slider:
                ii.Release(index)

    def Reset(self, names, gains):
        if len(names) != len(gains):
            raise 'you gave me %d names, but %d (sets of) gains' % (len(names), len(gains))
        for ii in self.gain_slider:
            ii.destroy()
        self.gain_slider = []
        for ii in xrange(len(self.master_var)):
            self.master_override[ii].set(0)
            self.master_var[ii].set(0)
        for ii in xrange(len(gains)):
            if len(self.prefixes) != len(gains[ii]):
                raise 'gain set number %d has %d entries but should have %d' % (ii, len(gains[ii]), len(self.prefixes))
            gs = GainSlider(self.gframe, self.prefixes, names[ii], self.maxvals, gains[ii])
            gs.pack(side = Tix.TOP, expand = True, fill = Tix.X)
            self.gain_slider.append(gs)
            
    def Read(self, index):
        if (index < 0) or (index >= len(self.master_var)):
            raise 'oops, index is %d but len(self.master_var) is %d' % (index, len(self.master_var))
        data = list()
        for ii in self.gain_slider:
            data.append(ii.GetGain(index))
        return data


class GainSetter(Tix.Frame):
    def __init__(self, parent, kpmax, kdmax):
        Tix.Frame.__init__(self, parent, bd = 2, relief = Tix.RIDGE)
        frame = Tix.Frame(self)
        self.button = Tix.Button(frame, text = 'SetGains', command = self.SetGains)
        self.button.pack(side = Tix.LEFT)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(no status yet)')
        self.label = Tix.Label(frame, textvariable = self.labeltext)
        self.label.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
        frame.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.gains = BunchOfGainSliders(self, ['kp', 'kd'], [kpmax, kdmax])
        self.gains.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.set_gains = None

    def Reset(self, names, kp, kd):
        if (len(names) != len(kp)) or (len(names) != len(kd)):
            raise 'oops, %d names and %d kps and %d kds... size mismatch' % (len(names), len(kp), len(kd))
        gains = []
        for ii in xrange(len(names)):
            gains.append([kp[ii], kd[ii]])
        self.gains.Reset(names, gains)
        
    def LazyInit(self):
        if not self.set_gains:
            self.labeltext.set('lazy init...')
            rospy.wait_for_service('/jspace_servo/set_gains')
            self.set_gains = rospy.ServiceProxy('/jspace_servo/set_gains', srv.SetGains)
            self.labeltext.set('lazy init DONE')
        
    def SetGains(self):
        try:
            self.LazyInit()
            kp = self.gains.Read(0)
            kd = self.gains.Read(1)
            response = self.set_gains(kp, kd)
            if not response.ok:
                self.labeltext.set('%s' % response.errstr)
            else:
                self.labeltext.set('ok')
        except rospy.ServiceException, ee:
            self.labeltext.set('EXCEPTION: %s' % ee)


class Initializer(Tix.Frame):
    def __init__(self, parent, goal_setter, gain_setter, controller_selector):
        Tix.Frame.__init__(self, parent)
        self.goal_setter = goal_setter
        self.gain_setter = gain_setter
        self.controller_selector = controller_selector
        self.button = Tix.Button(self, text = 'Initialize', bg = '#66cc66', command = self.Initialize)
        self.button.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.labeltext = Tix.StringVar(self)
        self.labeltext.set('(not initialized yet)')
        self.label = Tix.Label(self, textvariable = self.labeltext, anchor = 'w')
        self.label.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.get_info = None
        self.get_state = None
        
    def LazyInit(self):
        if not self.get_info:
            self.labeltext.set('lazy init...')
            rospy.wait_for_service('/jspace_servo/get_info')
            self.get_info = rospy.ServiceProxy('/jspace_servo/get_info', srv.GetInfo)
            rospy.wait_for_service('/jspace_servo/get_state')
            self.get_state = rospy.ServiceProxy('/jspace_servo/get_state', srv.GetState)
            self.labeltext.set('lazy init DONE')

    def Initialize(self):
        try:
            self.LazyInit()
            info = self.get_info()
            state = self.get_state()
            if not state.ok:
                raise 'GetState() failed: %s' % state.errstr
            self.labeltext.set('initialized')
            self.goal_setter.Reset(state.active_controller.dof_name,
                                   state.active_controller.dof_unit,
                                   state.goal,
                                   state.actual,
                                   state.active_controller.limit_lower,
                                   state.active_controller.limit_upper)
            self.gain_setter.Reset(state.active_controller.gain_name,
                                   state.active_controller.kp,
                                   state.active_controller.kd)
            all_controller_names = []
            for ii in info.controller_info:
                all_controller_names.append(ii.controller_name)
            self.controller_selector.Reset(state.active_controller.controller_name, all_controller_names)
        except rospy.ServiceException, ee:
            self.labeltext.set('EXCEPTION: %s' % ee)


class ControllerSelector(Tix.Frame):
    def __init__(self, parent, goal_setter, gain_setter):
        Tix.Frame.__init__(self, parent)
        self.goal_setter = goal_setter
        self.gain_setter = gain_setter
        self.mbtext = Tix.StringVar(self)
        self.mbtext.set('controller: (none selected)')
        self.mbtext = Tix.StringVar(self)
        self.mb = Tix.Menubutton(self, textvariable = self.mbtext, relief = Tix.RAISED)
        self.mb.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.menu = Tix.Menu (self.mb, tearoff = 0)
        self.mb["menu"] = self.menu
        self.statustext = Tix.StringVar(self)
        self.statustext.set('(no status yet)')
        self.status = Tix.Label(self, textvariable = self.statustext)
        self.status.pack(side = Tix.TOP, expand = True, fill = Tix.X)
        self.select_controller = None
        self.get_state = None
        
    def Reset(self, active_controller, controller_names):
        self.statustext.set('(no status yet)')
        self.mbtext.set('controller: %s' % active_controller)
        self.menu.delete(0, Tix.END)
        for nm in controller_names:
            self.menu.add_command(label = nm, command = partial(self.Select, nm))
        
    def LazyInit(self):
        if not self.select_controller:
            self.statustext.set('lazy init...')
            rospy.wait_for_service('/jspace_servo/select_controller')
            self.select_controller = rospy.ServiceProxy('/jspace_servo/select_controller', srv.SelectController)
            rospy.wait_for_service('/jspace_servo/get_state')
            self.get_state = rospy.ServiceProxy('/jspace_servo/get_state', srv.GetState)
            self.statustext.set('lazy init DONE')

    def Select(self, controller_name):
        try:
            self.LazyInit()
            response = self.select_controller(controller_name)
            if not response.ok:
                self.statustext.set('%s' % response.errstr)
            else:
                state = self.get_state()
                if not state.ok:
                    raise 'GetState() failed: %s' % state.errstr
                self.goal_setter.Reset(state.active_controller.dof_name,
                                       state.active_controller.dof_unit,
                                       state.goal,
                                       state.actual,
                                       state.active_controller.limit_lower,
                                       state.active_controller.limit_upper)
                self.gain_setter.Reset(state.active_controller.gain_name,
                                       state.active_controller.kp,
                                       state.active_controller.kd)
                self.statustext.set('ok')
                self.mbtext.set('controller: %s' % controller_name)
        except rospy.ServiceException, ee:
            self.statustext.set('EXCEPTION: %s' % ee)


if __name__ == '__main__':
    root = Tix.Tk()
    root.title('stanford-wbc mixer')
    
    lframe = Tix.Frame(root, bd = 2, relief = Tix.RIDGE)
    rframe = Tix.Frame(root)
    
    goal_setter = GoalSetter(rframe)
    gain_setter = GainSetter(rframe, 1000, 100)
    
    quitter = Tix.Button(lframe, text = 'QUIT', command = root.quit, bg = '#cc6666')
    controller_selector = ControllerSelector(lframe, goal_setter, gain_setter)
    initializer = Initializer(lframe, goal_setter, gain_setter, controller_selector)

    lframe.pack(side = Tix.LEFT, expand = True, fill = Tix.Y)
    rframe.pack(side = Tix.LEFT, expand = True, fill = Tix.BOTH)
    
    goal_setter.pack(side = Tix.LEFT, expand = True, fill = Tix.X)
    gain_setter.pack(side = Tix.LEFT, expand = True, fill = Tix.X)

    initializer.pack(side = Tix.TOP, expand = True, fill = Tix.X)
    controller_selector.pack(side = Tix.TOP, expand = True, fill = Tix.X)
    Tix.Frame(lframe).pack(side = Tix.TOP, expand = True, fill = Tix.BOTH)
    quitter.pack(side = Tix.TOP, expand = True, fill = Tix.X)
    
    root.mainloop()


def testGoalSlider():
    root = Tix.Tk()
    root.title('testGoalSlider')
    foo = GoalSlider(root, 'foo', 'm', -100, 0, -80, -60);
    foo.pack(expand = True, fill = Tix.X)
    bar = GoalSlider(root, 'bar', 'rad', -math.pi, math.pi, -0.1, 0.05);
    bar.pack(expand = True, fill = Tix.X)
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit)
    quitter.pack()
    root.mainloop()


def testBunchOfGoalSliders():
    root = Tix.Tk()
    root.title('testBunchOfGoalSliders')
    foo = BunchOfGoalSliders(root);
    foo.pack(expand = True, fill = Tix.X)
    foo.Reset(['toto', 'tutu'], ['rad', 'bananas'], [20, -20], [19, -22], [-100, -500], [50, 100])
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit)
    quitter.pack()
    root.mainloop()


def testGoalSetter():
    root = Tix.Tk()
    root.title('testGoalSetter')
    foo = GoalSetter(root);
    foo.pack(expand = True, fill = Tix.X)
    foo.Reset(['toto', 'tutu'], ['apples', 'radians'], [20, -20], [19, -22], [-100, -500], [50, 100])
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit)
    quitter.pack()
    root.mainloop()


def testGainSlider():
    root = Tix.Tk()
    root.title('testGainSlider')
    foo = GainSlider(root, ['kp', 'kd'], 'foo', [1000, 100], [100, 10]);
    foo.pack(expand = True, fill = Tix.X)
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit)
    quitter.pack()
    root.mainloop()


def testBunchOfGainSliders():
    root = Tix.Tk()
    root.title('testBunchOfGainSliders')
    foo = BunchOfGainSliders(root, ['kp', 'kd'], [1000, 100])
    foo.pack(expand = True, fill = Tix.X)
    foo.Reset(['toto', 'tutu'], [[200, 20], [333, 33]])
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit)
    quitter.pack()
    root.mainloop()


def testGainSetter():
    root = Tix.Tk()
    root.title('testGainSetter')
    foo = GainSetter(root, 1000, 80)
    foo.pack(expand = True, fill = Tix.X)
    foo.Reset(['toto', 'tutu'], [20, 100], [19, 22])
    quitter = Tix.Button(root, text = 'QUIT', command = root.quit)
    quitter.pack()
    root.mainloop()
