#!bin/bash

for i in $(ls $1)
do
	date=$(date -r $1/$i +%H%M%S);
	convert "$1/$i" "$1/$date.jpg" ; 
done;	