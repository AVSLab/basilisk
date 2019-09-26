for file in *.tex
do
    echo ${file}
    sourceFile=${file}
    outputFile=${sourceFile%.tex}
    outputFile+=".rst"
    pandoc -s -t rst --toc ${sourceFile} -o ${outputFile}
done
