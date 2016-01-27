DATA_IN_DIR_FULLPATH="/media/yfang/hdd/ControllerData/runrecord2_20150914_newprocessdelay_next150/"
DBNAME="simtest200_b"
counter=0

echo $DBNAME
for D in `find $DATA_IN_DIR_FULLPATH -type d`
do
	filepath=$D"/state.log" #the loop goes through every subfolder so at some point it will be at the right level
	#collectionname=`echo $filepath | cut -d '_' -f 4``echo $filepath | cut -d '_' -f 5`
	re="\/.*\/.*\/(.*)\/gzserver\/state\.log"
	if [[ $filepath =~ $re ]]; then 

		collectionname=${BASH_REMATCH[1]};
		collectionname=${collectionname//[^[:alnum:]]/}
		echo $filepath" to "$collectionname
		gzserver --verbose -s libPostProcess.so -p $filepath --db $DBNAME --collection $collectionname --suffix $counter -replaying
		counter=$((counter+1))
	fi	
done
