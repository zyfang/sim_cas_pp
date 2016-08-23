# RUN_NAME="A_yf1"
EXP_NAME="run1"
DATA_IN_DIR_FULLPATH="/media/yfang/hdd3/HumanData/Robohow1/logs_drink/"$EXP_NAME"/"
EXCLUSION_FILE="/home/yfang/workspaces/catkin_ws/src/sim_cas_pp/bad_demos.csv"

for D in `find $DATA_IN_DIR_FULLPATH -type d`
do
	filepath=$D"/state.log"
	#collectionname=`echo $filepath | cut -d '_' -f 4``echo $filepath | cut -d '_' -f 5`
	re="$EXP_NAME\/(.+)\_(.+)\/(.+)\/.+\/gzserver\/state\.log"
	if [[ $filepath =~ $re ]]; then 

		order=${BASH_REMATCH[1]};
		subj=${BASH_REMATCH[2]};
		collectionname=${BASH_REMATCH[3]};
		while IFS=, read col1 col2 col3
		do
			if [[ $order == $col1 ]] && [[ $subj == $col2 ]] && [[ $collectionname == $col3 ]]; then
				echo $order\_$subj\_$collectionname" skipped because in bad_demos.csv"
			else
				gzserver --verbose -s libPostProcess.so -p $filepath --config config_drink.cfg --db $EXP_NAME --collection $order\_$subj\_$collectionname --suffix 0 --delay 2 -replaying 
			fi	
		done < $EXCLUSION_FILE

		# echo $filepath" to "$order\_$subj\_$collectionname
		# gzserver --verbose -s libPostProcess.so -p $filepath --config config_drink.cfg --db $EXP_NAME --collection $order\_$subj\_$collectionname --suffix 0 --delay 2 -replaying 
	fi	
done
