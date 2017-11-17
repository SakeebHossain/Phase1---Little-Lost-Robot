#!/usr/bin/perl

use LEGO::NXT;
use LEGO::NXT::BlueComm;
use LEGO::NXT::Constants qw(:DEFAULT);
use Data::Dumper;
use Net::Bluetooth;
use strict;


my $nxt = LEGO::NXT->new('00:16:53:03:96:6B', 1);

$nxt->initialize_ultrasound_port($NXT_SENSOR_4);

@bla = $nxt->get_ultrasound_measurement_units($NXT_SENSOR_4);

print "bla is $bla\n";
