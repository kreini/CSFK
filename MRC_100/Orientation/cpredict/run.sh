#!/bin/bash

function satellite_xyz()
{
 ./cpredict -f grbalpha.tle -t $1 | grep ^ECI_[xyz] | awk '{ print $2; }'
}

function sun_xyz()
{
 ./cpredict -f grbalpha.tle -t $1 | grep ^ECI_sun_[xyz] | awk '{ print $2; }'
}

echo 'time ECI_x ECI_y ECI_z ECI_sun_x ECI_sun_y ECI_sun_z'

for t in `seq 1625426000 250000 1656962000`; do
	echo $t `satellite_xyz $t` `sun_xyz $t`
done
