pdf:
	@mkdir -p build/
	@mkdir -p build/images/
	@mkdir -p build/chapters
	@latexmk -shell-escape -pdf main.tex
	@\cp build/main.pdf .

clean:
	@latexmk -C
	@rm -r build/images/
	@echo "--------------------------------------------------------------------------"
	@echo " Cleaned build files !                                                    "
	@echo "--------------------------------------------------------------------------"

all: pdf
	@git submodule update
	@echo "--------------------------------------------------------------------------"
	@echo " Updated code files (Reminder : Make sure to update the code lines marker "
	@echo "     with \inputminted commands, or some functions may now be incomplete!)"
	@echo "                                                                          "
	@echo " Generated PDF files and copied into the base folder !"
	@echo "--------------------------------------------------------------------------"

all+clean: all
	@latexmk -c

install:
	@cd data/code & git submodule add https://github.com/lheywang/Opale.git

