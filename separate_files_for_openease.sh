## iterate through each file whose name ends in 'jpg'
## saving it as $file. ~ is your $HOME directory
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

    ## rename tf files to only "tf"
    for d in `find . -mindepth 1 -type d` #mindepth so it doesn't list the current directory
    do
    	# echo $d
    	mv $d"/"*"_tf.bson" $d"/tf.bson" 
    	mv $d"/"*".owl" $d"/log.owl" 
    	mv $d"/"*".html" $d"/timeline.html" 
    	mv $d"/"*"_ev.bson" $d"/ev.bson" 
    done
    
done