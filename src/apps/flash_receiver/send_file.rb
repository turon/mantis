#!/usr/bin/ruby

require 'serial_utils'
require 'semaphore'

include SerialUtils

PREAMBLE_BYTE = 'S'
PREAMBLE_COUNT = 2
ACK_PACKET = ":)"

@dev_name = ARGV[0]
sem = Semaphore.new()

puts "using device: #{@dev_name}"

@dev = setup_port(@dev_name)

t1 = Thread.start(@dev) do | dev | 
  begin
    preamble_count = 0
    loop do
      if @dev.read(1) == PREAMBLE_BYTE
        preamble_count += 1
        if preamble_count == PREAMBLE_COUNT
          preamble_count = 0
          str = @dev.read(@dev.read(1)[0])
          sem.signal if str == ACK_PACKET
          
          print str
          $stdout.flush
        end
      end
    end
  rescue => er
    puts "listening error: " + er
  end
end


def read_message()
  preamble_count = 0
  if @dev.read(1) == PREAMBLE_BYTE
      preamble_count += 1
      if preamble_count == PREAMBLE_COUNT
        preamble_count = 0
        return @dev.read(@dev.read(1)[0])
        # $stdout.flush
      end
    end
end

def send_message(message)
  #message.gsub!(/\n/,"")
  data = ["S","S", message.length].pack("AAC") + message
  @dev.write(data)
  @dev.flush
end

file_name = $stdin.gets

file = File.new(file_name.chomp, "rb");
line = file.read(File.size(file));
file_size = File.size(file)

file_size_str =[file_size].pack("L")

send_message(file_size_str)
sem.wait
#line = line[6...line.length]

#t2 = Thread.start {
while line.length > 64
   #puts "sending line"
   send_message(line[0...64])
   sem.wait
   line = line[64...line.length]
end

if line.length != 0
   #puts "sending last line"
   send_message(line)
end# }

t1.join
t2.join
