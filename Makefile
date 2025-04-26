pdf:
	@mkdir -p build/
	@mkdir -p build/images/
	@mkdir -p build/chapters
	git submodule update
	latexmk -shell-escape -pdf main.tex

clean:
	@latexmk -C
	@rm -r build/images/
	@echo "--------------------------------------------------------------------------"
	@echo " Cleaned build files !                                                    "
	@echo "--------------------------------------------------------------------------"

all: pdf
	@echo "--------------------------------------------------------------------------"
	@echo " Generated .pdf files into the build/ folder !                   "
	@echo "--------------------------------------------------------------------------"

all+clean: all
	@latexmk -c

install:
	@cd data/code & git submodule add https://github.com/lheywang/Opale.git

