DATA_IN_DIR_FULLPATH="/media/yfang/hdd/HumanData/ExperimentSubjectsData/ferko_run_1/"
DBNAME="ferko_run_1"
counter=0

echo $DBNAME
for D in `find $DATA_IN_DIR_FULLPATH -type d`
do
	filepath=$D"/state.log" #the loop goes through every subfolder so at some point it will be at the right level
	#collectionname=`echo $filepath | cut -d '_' -f 4``echo $filepath | cut -d '_' -f 5`
	re="\/.*\/.*\/.*_(mug.*_.*)\/gzserver\/state\.log"
	if [[ $filepath =~ $re ]]; then 

		collectionname=${BASH_REMATCH[1]};
		#echo $filepath" to "$collectionname
		gzserver --verbose -s libPostProcess.so -p $filepath --db $DBNAME --collection $collectionname --suffix $counter -replaying
		counter=$((counter+1))
	fi	
done
