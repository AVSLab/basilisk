for file in *.md
do
    echo ${file}
    sourceFile=${file}
    outputFile=${sourceFile%.md}
    outputFile+=".rst"
    pandoc -s -t rst --toc ${sourceFile} -o ${outputFile}
done
