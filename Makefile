pdf:
	@mkdir -p build/
	@mkdir -p build/images/
	@mkdir -p build/chapters
	@latexmk -shell-escape -pdf main.tex
	@\cp build/main.pdf .

compress: pdf
	@gs \
		-dNOPAUSE \
		-dBATCH \
		-dSAFER \
		-sDEVICE=pdfwrite \
		-dCompatbilityLevel=1.4 \
		-dPDFSETTINGS=/prepress \
		-dDetectDuplicateImages=true \
		-dCompressFonts=true \
		-sOutputFile=build/main_compressed.pdf \
		build/main.pdf

clean:
	@latexmk -C
	@rm -r build/images/
	@echo "--------------------------------------------------------------------------"
	@echo " Cleaned build files !                                                    "
	@echo "--------------------------------------------------------------------------"

all: compress
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
	git clone --recursive https://github.com/lheywang/Opale-Report.git Opale-Report

