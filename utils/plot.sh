#!/bin/bash
gnuplot -persist <<-EOF
	set term x11
	set datafile separator ","
	set xlabel "Time"
	set ylabel "RPM"
	plot "$1" using 1:10 with lines title "r\_rpm", "$1" using 1:12 with lines title "r\_rpm\_ref"
	reread