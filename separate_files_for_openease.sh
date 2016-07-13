## Seperates bson and owl files into different directories such that they can be used for openEASE (one directory per episode)
for file in /media/yfang/hdd/docker/episodes/Pancake-Making/sim-pour_0/*_ev.bson
do
    ## basename will remove the path (~/Desktop/My_pictures) and also
    ## remove the extension you give as a second argument    
    name="$(basename "$file" _ev.bson)"

    # echo $name

    ## create the directory, the -p means it will create 
    ## the parent directories if needed
    mkdir -p /media/yfang/hdd/docker/episodes/Pancake-Making/sim-pour_0/"$name"

    ## copy the file to the new directory
    #mv "$file" "~/Desktop/My_pictures/$name"
    mv $(find . -type f -name "*$name*") /media/yfang/hdd/docker/episodes/Pancake-Making/sim-pour_0/"$name"/

    ## rename tf files to only "tf", etc.
    for d in `find . -mindepth 1 -type d` #mindepth so it doesn't list the current directory
    do
    	# echo $d
    	mv $d"/"*"_tf.bson" $d"/tf.bson" 
    	mv $d"/"*".owl" $d"/log.owl" 
    	# mv $d"/"*".html" $d"/timeline.html" #Not necessary anymore since I implemented the timeline generation code
    	mv $d"/"*"_ev.bson" $d"/ev.bson" 
    done
    
done