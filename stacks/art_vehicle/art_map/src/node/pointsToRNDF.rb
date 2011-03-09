#!/usr/bin/ruby -w
# Code by Alex Menzies
# 2.21.07

# Note that the first line in this file is nessessary to make this 
# script executable.  It must contain the path of the local systems
# ruby interpreter (e.g /usr/bin/ruby).  This path can be found by 
# running "which ruby"

# This script is designed to take output from the getpoints cleint and convert it into a
# simple RNDF file.  This file can then be edited using the visualRNDF editor utility.
# > ./rddf_to_rndf.rb <input_file_name> <output_file_name>
# Where the input file is comes from getpoints.  An RNDF file will be created
# using the specified output name and will overwrite any pre-existing file with that name

class Point
  def initialize(lat,lng)
    @lat = lat
    @lng = lng
  end
  attr_accessor :lat, :lng
end

if __FILE__ == $0
  #The following code will only be executed when
  #this file is executed from the command line

  # Raise an exception if we didn't get two command line arguments
  raise "Input and Output files must be specified on command line" if ARGV.size != 2
  input_file_name = ARGV[0];
  output_file_name = ARGV[1];

  lanes = []
  zones = []
  cur_section = nil


  # Read RDDF
  File.open(input_file_name) do |input|
    while(line = input.gets)
      case line
        when /file_start/ then
        when /file_end/ then
        when /end/ then
#          puts "end"
          cur_section = nil
        when /lane_point\s+([-]?\d+[.]?\d+)\s+([-]?\d+[.]?\d+)/
          lanes << [] if cur_section == nil
          lanes.last << Point.new($1, $2)
#          puts "lane #{$1} #{$2}"
          cur_section = :lane
        when /perim_point\s+([-]?\d+[.]?\d+)\s+([-]?\d+[.]?\d+)/
          zones << [[],[]] if cur_section == nil
          perims, spots = zones.last
          perims << Point.new($1, $2)
#          puts "perim #{$1} #{$2}"
          cur_section = :zone
        when /spot_point\s+([-]?\d+[.]?\d+)\s+([-]?\d+[.]?\d+)/
          zones << [[],[]] if cur_section == nil
          perims, spots = zones.last
          spots << Point.new($1, $2)
#          puts "spot #{$1} #{$2}"
          cur_section = :zone
        else
          puts "Syntax Error: #{line}"
      end
       
    end
  end

  # Write RNDF
  File.open(output_file_name, "w") do |output|
    output.puts "RNDF_name\t#{output_file_name.split('.')[0].strip}" #remove extension and whitespace
    output.puts "num_segments\t#{lanes.length}"
    output.puts "num_zones\t#{zones.length}"
    lanes.each_with_index do |lane, index|
      lane_num = index + 1
      output.puts "segment\t#{lane_num}"
      output.puts "num_lanes\t1"
      output.puts "lane\t#{lane_num}.1"
      output.puts "num_waypoints\t#{lane.length}"
      lane.each_with_index do |pt, i|
        output.puts "#{lane_num}.1.#{i+1}\t#{pt.lat}\t#{pt.lng}"
      end
      output.puts "end_lane"
      output.puts "end_segment"
    end
    zones.each_with_index do |zone, index|
      zone_num = index + lanes.length + 1
      perim, spots = zone
      output.puts "zone\t#{zone_num}"
      output.puts "num_spots\t#{(spots.length + 1) / 2}"
      output.puts "perimeter\t#{zone_num}.0"
      output.puts "num_perimeterpoints\t#{perim.length}"
      perim.each_with_index do |pt, i|
        output.puts "#{zone_num}.0.#{i+1}\t#{pt.lat}\t#{pt.lng}"
      end
      output.puts "end_perimeter"
      spots.each_with_index do |pt, i|
        spot_num = (i / 2) + 1;
        output.puts "spot\t#{zone_num}.#{spot_num}" if i % 2 == 0
        output.puts "#{zone_num}.#{spot_num}.#{(i%2)+1}\t#{pt.lat}\t#{pt.lng}"
        output.puts "end_spot" unless i % 2 == 0
      end
      output.puts "end_spot" unless spots.length % 2 == 0
      output.puts "end_zone"
    end
    output.puts "end_file"
  end
end
