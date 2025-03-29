pdf:
	latexmk -shell-escape -pdf main.tex

clean:
	@latexmk -C
	@echo "--------------------------------------------------------------------------"
	@echo " Cleaned build files !                                                    "
	@echo "--------------------------------------------------------------------------"

all: pdf
	@echo "--------------------------------------------------------------------------"
	@echo " Generated .pdf files into the build/ folder !                   "
	@echo "--------------------------------------------------------------------------"

all+clean: all
	@latexmk -c
