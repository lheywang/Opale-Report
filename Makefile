dvi:
	latexmk -shell-escape -dvi main.tex

pdf:
	latexmk -shell-escape -pdf main.tex

clean:
	@latexmk -C
	@echo "--------------------------------------------------------------------------"
	@echo " Cleaned build files !                                                    "
	@echo "--------------------------------------------------------------------------"

all: pdf dvi
	@mkdir -p build
	@cp main.pdf build/.
	@cp main.dvi build/.
	@echo "--------------------------------------------------------------------------"
	@echo " Generated .dvi and .pdf files into the build/ folder !                   "
	@echo "--------------------------------------------------------------------------"

all+clean: all clean

