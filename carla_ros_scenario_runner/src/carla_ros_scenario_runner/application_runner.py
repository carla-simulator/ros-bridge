#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
"""
Execute an application.
"""
import os
from enum import Enum
from threading import Thread, Event
from datetime import datetime, timedelta
import pexpect


class ApplicationStatus(Enum):
    """
    States of an application
    """
    STOPPED = 0
    STARTING = 1
    RUNNING = 2
    SHUTTINGDOWN = 3
    ERROR = 4


class ApplicationRunner(object):

    """
    Execute application
    """

    def __init__(self, status_updated_fct, log_fct, ready_string=""):
        """
        Constructor
        """
        self._app_thread = None
        self._status_updated_fct = status_updated_fct
        self._log_fct = log_fct
        self._shutdown_requested_event = Event()
        self._ready_string = ready_string

    def execute(self, cmdline, env=None, cwd=None):
        """
        Starts a thread to execute the application
        """
        if self.is_running():
            self._log_fct("Application already running!")
            return False

        self._shutdown_requested_event.clear()
        self._app_thread = Thread(target=self.start_and_run, args=(cmdline,
                                                                   env,
                                                                   cwd,
                                                                   self._shutdown_requested_event,
                                                                   self._ready_string,
                                                                   self._status_updated_fct,
                                                                   self._log_fct,))
        self._app_thread.start()

        return True

    def start_and_run(self, cmdline, env, cwd, shutdown_requested_event, ready_string,  # pylint: disable=too-many-arguments
                      status_updated_fct, log_fct):
        """
        thread function
        """
        status_updated_fct(ApplicationStatus.STARTING)
        try:
            process = self.start_process(cmdline, log_fct, env=env, cwd=cwd)
            self.run(process, shutdown_requested_event, ready_string, status_updated_fct, log_fct)
        except (KeyError, pexpect.ExceptionPexpect) as e:
            self._log_fct("Error while starting process: {}".format(e))
            status_updated_fct(ApplicationStatus.ERROR)

    def is_running(self):
        """
        returns if the application is still running
        """
        if self._app_thread is None:
            return False

        return self._app_thread.is_alive()

    def shutdown(self):
        """
        Shut down the application thread
        """
        if not self.is_running():
            return
        self._log_fct("Requesting shutdown...")
        self._status_updated_fct(ApplicationStatus.SHUTTINGDOWN)
        self._shutdown_requested_event.set()
        if self._app_thread:
            self._app_thread.join()
        self._log_fct("Shutdown finished.")

    def start_process(self, argument_list, log_fct, env=None, cwd=None):  # pylint: disable=no-self-use
        """
        Starts a process.
        """
        if not argument_list:
            raise KeyError("No arguments given!")
        executable = argument_list[0]
        if not os.path.isfile(executable):
            raise KeyError("The executable {} does not exist".format(executable))
        log_fct("Executing: {}".format(" ".join(argument_list)))
        process = pexpect.spawn(" ".join(argument_list), env=env, cwd=cwd)
        #process.logfile_read = sys.stdout
        return process

    def run(self, process, shutdown_requested_event, ready_string, status_updated_fct, log_fct):  # pylint: disable=no-self-use,too-many-arguments
        """
        Threaded application execution

        :return:
        """
        shutting_down_trigger_time = None
        signaled_running = False
        while True:
            if shutdown_requested_event.is_set():
                if shutting_down_trigger_time is None:
                    shutting_down_trigger_time = datetime.now()
                    log_fct("Shutdown requested while process is still \
                        running. Sending SIGHUP/SIGINT...")
                    process.terminate(force=False)
                else:
                    if (datetime.now() - shutting_down_trigger_time) > timedelta(seconds=8):
                        log_fct("Waited 8s for application to exit. Forcing Shutdown. \
                            Sending SIGKILL")
                        process.terminate(force=True)
            try:
                process.expect(".*\n", timeout=0.1)
                log_fct(process.after.strip())
                if not signaled_running:
                    if process.after.find(ready_string) != -1:
                        status_updated_fct(ApplicationStatus.RUNNING)
                        log_fct("Application is ready.")
                        signaled_running = True
            except pexpect.EOF:
                # application exited
                log_fct(process.before.strip())
                log_fct("Application exited. Exiting run loop")
                break
            except pexpect.TIMEOUT:
                # no output received
                pass

        process.close()
        if process.exitstatus == 0:
            status_updated_fct(ApplicationStatus.STOPPED)
        else:
            status_updated_fct(ApplicationStatus.ERROR)
