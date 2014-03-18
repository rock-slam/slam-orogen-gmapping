require 'orocos'
require 'orocos/log'
require 'vizkit'
require 'transformer/runtime'

include Orocos
Orocos.initialize

# Location of the log files
replay = Log::Replay.open('/home/krishna/bagfiles/rock_log/aria.1.log','/home/krishna/bagfiles/rock_log/hokuyo.1.log')

Orocos.run 'gmapping::Task' => 'gmapping' do
  
  gmapping = Orocos.name_service.get('gmapping')
  aria = Orocos.name_service.get('aria')

  replay.aria.robot_pose_raw.connect_to  gmapping.dynamic_transformations
  replay.hokuyo.scans.connect_to gmapping.scan

  gmapping.apply_conf_file("../config.yml")

  gmapping.configure
  
  gmapping.start
  puts "task started.."
  Vizkit.control replay
  Vizkit.exec

  
end
