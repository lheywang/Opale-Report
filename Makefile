dvi:
	latexmk -shell-escape -dvi main.tex

pdf:
	latexmk -shell-escape -pdf main.tex

clean:
	latexmk -C

all: pdf dvi
	mkdir -p build
	cp main.pdf build/.
	cp main.dvi build/.

all+clean: all clean

