#!/usr/bin/ruby

require 'fcntl'
require 'termios'
module SerialUtils
  def setup_port(devname)
    dev = open(devname, File::RDWR | File::NONBLOCK)
    
    #get old settings
    mode = dev.fcntl(Fcntl::F_GETFL, 0)
    
    dev.fcntl(Fcntl::F_SETFL, mode & ~File::NONBLOCK)
    
    tio = Termios::new_termios
    tio.iflag = 0
    tio.oflag = 0
    tio.cflag = Termios::CS8 | Termios::CREAD | Termios::CLOCAL
    tio.lflag = 0
    tio.cc[Termios::VTIME] = 0
    tio.cc[Termios::VMIN] = 1
    tio.ispeed = tio.ospeed = Termios::B57600
    Termios::flush(dev, Termios::TCIOFLUSH)
    Termios::setattr(dev, Termios::TCSANOW, tio)
    return dev
  end
  
  def wait_for_timeout(timeout,time_timed_out)
    ret = select([@dev], nil, nil,timeout)
    if ret.nil? then
      if was_timed_out == false or times_timed_out < 3 then
        times_timed_out += 1
        printf "timeout, f: #{was_timed_out}, m: #{failure_mode}\n"
        if failure_mode > 1 then
          restart_node
          @dev = setup_port(@dev_name)
        elsif failure_mode > 0 then
          handle_message("SYSTEM: Node timed out after #{timeout} seconds.")
        end
        was_timed_out = true
      end
      next
    else
      was_timed_out = false
      times_timed_out = 0
    end
    return time_timed_out
  end
end
