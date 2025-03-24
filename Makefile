dvi:
	latexmk -shell-escape -dvi main.tex

pdf:
	latexmk -shell-escape -pdf main.tex

clean:
	latexmk -c

all: pdf dvi

all+clean: pdf dvi clean
