DATA_IN_DIR_FULLPATH="/media/yfang/hdd3/HumanData/Robohow1/logs_drink/robohow1/A_yf1/"
DBNAME="A_yf1"
echo $DBNAME
for D in `find $DATA_IN_DIR_FULLPATH -type d`
do
	filepath=$D"/state.log"
	#collectionname=`echo $filepath | cut -d '_' -f 4``echo $filepath | cut -d '_' -f 5`
	re="\/$DBNAME\/(.*)\/.*\/gzserver\/state\.log"
	if [[ $filepath =~ $re ]]; then 

		collectionname=${BASH_REMATCH[1]};
		# echo $filepath" to "$collectionname
		gzserver --verbose -s libPostProcess.so -p $filepath --db $DBNAME --collection $collectionname
	fi	
done
