OPTIONS=-shell-escape

all:
	pdflatex report.tex 

clean:
	rm -f *.aux *.log
	rm -f report.pdf	

view: all
	evince report.pdf&
