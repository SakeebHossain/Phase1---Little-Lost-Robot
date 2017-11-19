use LEGO::NXT;
use LEGO::NXT::BlueComm;
use LEGO::NXT::Constants qw(:DEFAULT);
use Data::Dumper;
use Net::Bluetooth;
use strict;

my $addr = $ARGV[0];
my $port = 1;

die "No Bluetooth Address Specified!\n" if !$addr;

$| = 1;

my $res;

my $bt = LEGO::NXT->new( new LEGO::NXT::BlueComm($addr,$port) );

$res = $bt->play_sound_file($NXT_RET, 0,'! Attention.rso');
print Dumper($res);

$res  = $bt->get_battery_level($NXT_RET);
print Dumper($res);

$bt->initialize_ultrasound_port($NXT_SENSOR_4);
$res = $bt->get_ultrasound_measurement_units($NXT_SENSOR_4);
print Dumper($res);

exit;

#Also try these!
#
#$res = $bt->play_tone($NXT_RET,220*2,500);
#$bt->set_output_state($NXT_NORET, $NXT_MOTOR_A, 100, $NXT_MOTORON|$NXT_REGULATED, $NXT_REGULATION_MODE_MOTOR_SPEED, 0, $NXT_MOTOR_RUN_STATE_RUNNING, 0  );

