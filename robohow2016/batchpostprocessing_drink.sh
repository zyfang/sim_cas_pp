RUN_NAME="A_yf1"
EXP_NAME="param_test"
DATA_IN_DIR_FULLPATH="/media/yfang/hdd3/HumanData/Robohow1/logs_drink/"$EXP_NAME"/"

echo $RUN_NAME
for D in `find $DATA_IN_DIR_FULLPATH -type d`
do
	filepath=$D"/state.log"
	#collectionname=`echo $filepath | cut -d '_' -f 4``echo $filepath | cut -d '_' -f 5`
	re="\/$RUN_NAME\/(.*)\/.*\/gzserver\/state\.log"
	if [[ $filepath =~ $re ]]; then 

		collectionname=${BASH_REMATCH[1]};
		# echo $filepath" to "$collectionname
		gzserver --verbose -s libPostProcess.so -p $filepath --config config_drink.cfg --db $EXP_NAME\_$RUN_NAME --collection $collectionname --suffix 0 --delay 2 -replaying 
	fi	
done
