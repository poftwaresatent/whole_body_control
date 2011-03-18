#define RT_TASK_FREQUENCY_TORQUE_SHM 400
#define RT_TIMER_TICKS_NS_TORQUE_SHM (1000000000 / RT_TASK_FREQUENCY_TORQUE_SHM)

#define TORQUE_SHM "TSHMM"
#define TORQUE_CMD_SEM "TSHMC"
#define TORQUE_STATUS_SEM "TSHMS"


namespace wbc_m3_ctrl {
  
  struct rt_cb_s {
    rt_cb_s(): init(0), update(0), cleanup(0), slowdown(0) {}
    
    int (*init)(M3Sds * sys, );
    int (*update)(jspace::State const & state, jspace::Vector & command);
    int (*cleanup)(void);
    int (*slowdown)(long long iteration,
		    long long desired_ns,
		    long long actual_ns);
  };
  
  
  static enum {
    RT_THREAD_INIT,
    RT_THREAD_RUNNING,
    RT_THREAD_CLEANUP,
    RT_THREAD_ERROR,
    RT_THREAD_DONE
  } rt_thread_state;
  

  static int shutdown_request;
  
  
  static void * rt_thread(void * arg)
  {
    rt_thread_state = RT_THREAD_INIT;
    shutdown_request = 0;
    
    //////////////////////////////////////////////////
    // Initialize shared memory, RT task, and semaphores.
    
    M3Sds * sys;
    if (sys = (M3Sds*) rt_shm_alloc(nam2num(TORQUE_SHM), sizeof(M3Sds), USE_VMALLOC)) {
      printf("found shared memory\n");
    }
    else {
      printf("rt_shm_alloc failed for %s\n", TORQUE_SHM);
      rt_thread_state = RT_THREAD_ERROR;
      goto cleanup_sys;
    }
    
    RT_TASK * task;
    task = rt_task_init_schmod(nam2num("TSHMP"), 0, 0, 0, SCHED_FIFO, 0xF);
    rt_allow_nonroot_hrt();
    if (0 == task) {
      printf("rt_task_init_schmod failed for TSHMP\n");
      rt_thread_state = RT_THREAD_ERROR;
      goto cleanup_task;
    }
    
    SEM * status_sem;
    status_sem = (SEM*) rt_get_adr(nam2num(TORQUE_STATUS_SEM));
    if (status_sem) {
      printf("semaphore %s not found\n", TORQUE_STATUS_SEM);
      rt_thread_state = RT_THREAD_ERROR;
      goto cleanup_status_sem;
    }
    
    SEM * command_sem;
    command_sem = (SEM*) rt_get_adr(nam2num(TORQUE_CMD_SEM));
    if (!command_sem) {
      printf("semaphore %s not found\n", TORQUE_CMD_SEM);
      rt_thread_state = RT_THREAD_ERROR;
      goto cleanup_command_sem;
    }
    
    //////////////////////////////////////////////////
    // Give the user a chance to do stuff before we enter periodic
    // hard real time.
    
    rt_cb_s * rt_cb((rt_cb_s*) arg);
    if (rt_cb->init) {
      int const status(rt_cb->init(sys));
      if (0 != status) {
	printf("init callback returned %d\n", status);
	rt_thread_state = RT_THREAD_ERROR;
	goto cleanup_init_callback;
      }
    }
    
    //////////////////////////////////////////////////
    // Start the real time engine...

    M3TorqueShmSdsStatus shm_status;
    M3TorqueShmSdsCommand shm_cmd;
    
    jspace::State state(7, 7, 0); // XXXX to do: use all joints, add torques
    jspace::Vector command(7);
    
    rt_thread_state = RT_THREAD_RUNNING;
    RTIME tick_period(nano2count(RT_TIMER_TICKS_NS_TORQUE_SHM));
    rt_task_make_periodic(task, rt_get_time() + tick_period, tick_period); 
    mlockall(MCL_CURRENT | MCL_FUTURE);
    rt_make_hard_real_time();
    
    //////////////////////////////////////////////////
    // The loop.
    
    while (long long step_cnt(0); 0 == shutdown_request; ++step_cnt) {
      
      rt_task_wait_period();
      long long const start_time(nano2count(rt_get_cpu_time_ns()));
      
      rt_sem_wait(status_sem);
      memcpy(&shm_status, sds->status, sizeof(shm_status));
      rt_sem_signal(status_sem);
      for (size_t ii(0); ii < 7; ++ii) { // XXXX to do: hardcoded NDOF
	state.position_[ii] = M_PI * shm_status.right_arm.theta[ii] / 180.0;
	state.velocity_[ii] = M_PI * shm_status.right_arm.thetadot[ii] / 180.0;
      }
      
      if (rt_cb->update) {
	int const status(rt_cb->update(state, command));
	if (0 != status) {
	  printf("update callback returned %d\n", status);
	  rt_thread_state = RT_THREAD_ERROR;
	  shutdown_request = 1;
	  continue;
	}
      }
      
      for (size_t ii(0); ii < 7; ++ii) { // XXXX to do: hardcoded NDOF
	shm_cmd.right_arm.tq_desired[ii] = 1.0e3 * command[ii];
      }
      shm_cmd.timestamp = shm_status.timestamp;
      rt_sem_wait(command_sem);
      memcpy(sds->cmd, &shm_cmd, sizeof(shm_cmd));		
      rt_sem_signal(command_sem);
      
      long long const end_time(nano2count(rt_get_cpu_time_ns()));
      long long const dt(end_time - start_time);
      if (dt > tick_period) {
	if (rt_cb->slowdown) {
	  int const status(rt_cb->slowdown(step_cnt,
					   count2nano(tick_period),
					   count2nano(dt)));
	  if (0 != status) {
	    printf("slowdown callback returned %d\n"
		   "  iteration: %lld\n"
		   "  desired period: %lld ns\n"
		   "  actual period: %lld ns\n",
		   status, step_cnt, count2nano(tick_period), count2nano(dt));
	    rt_thread_state = RT_THREAD_ERROR;
	    shutdown_request = 1;
	    continue;
	  }
	}
	else if (10 < step_cnt) {
	  continue;
	}
	printf("slowing RT task down to %lld ns (instead of %lld ns)\n",
	        count2nano(tick_period), count2nano(dt));
	tick_period = dt;
	rt_task_make_periodic(task, end + tick_period, tick_period);			
      }
      
    } // end "the big while loop"

    //////////////////////////////////////////////////
    // Clean up after ourselves
    
    printf("exiting RT thread\n");
    
    rt_thread_state = RT_THREAD_CLEANUP;
    rt_make_soft_real_time();
    
    if (rt_cb->cleanup) {
      int const status(rt_cb->cleanup());
      if (0 != status) {
	printf("cleanup callback returned %d\n", status);
	rt_thread_state = RT_THREAD_ERROR;
      }
      else {
	rt_thread_state = RT_THREAD_DONE;
      }
    }
    else {
      rt_thread_state = RT_THREAD_DONE;
    }
    
  cleanup_init_callback:
  cleanup_command_sem:
  cleanup_status_sem:
    rt_task_delete(task);
  cleanup_task:
    rt_shm_free(nam2num(TORQUE_SHM));
  cleanup_sys:
    
    return 0;
  }
  
}
