# Makefile for PX4 Real-Time Mathematical Proofs LaTeX Document

# Variables
MAIN = PX4_Real_Time_Mathematical_Proofs
TEXFILE = $(MAIN).tex
PDFFILE = $(MAIN).pdf
BIBFILE = references.bib

# LaTeX compiler
LATEX = pdflatex
BIBTEX = bibtex

# Default target
all: $(PDFFILE)

# Build the PDF
$(PDFFILE): $(TEXFILE) $(BIBFILE)
	@echo "Compiling LaTeX document..."
	$(LATEX) $(TEXFILE)
	$(BIBTEX) $(MAIN)
	$(LATEX) $(TEXFILE)
	$(LATEX) $(TEXFILE)
	@echo "PDF compilation complete: $(PDFFILE)"

# Clean auxiliary files
clean:
	@echo "Cleaning auxiliary files..."
	rm -f *.aux *.log *.bbl *.blg *.toc *.out *.fdb_latexmk *.fls *.synctex.gz

# Clean all generated files including PDF
cleanall: clean
	@echo "Cleaning all generated files..."
	rm -f $(PDFFILE)

# View the PDF (macOS)
view: $(PDFFILE)
	@echo "Opening PDF..."
	open $(PDFFILE)

# Quick build (without bibliography)
quick: $(TEXFILE)
	@echo "Quick compilation (no bibliography)..."
	$(LATEX) $(TEXFILE)

# Count words in the document
wordcount: $(TEXFILE)
	@echo "Word count:"
	@texcount -brief $(TEXFILE)

# Check for LaTeX errors
check: $(TEXFILE)
	@echo "Checking LaTeX syntax..."
	$(LATEX) -draftmode -halt-on-error $(TEXFILE)

# Help target
help:
	@echo "Available targets:"
	@echo "  all       - Build the PDF document (default)"
	@echo "  clean     - Remove auxiliary files"
	@echo "  cleanall  - Remove all generated files including PDF"
	@echo "  view      - Open the PDF (macOS)"
	@echo "  quick     - Quick compilation without bibliography"
	@echo "  wordcount - Count words in the document"
	@echo "  check     - Check LaTeX syntax"
	@echo "  help      - Show this help message"

# Phony targets
.PHONY: all clean cleanall view quick wordcount check help

# Dependencies
$(TEXFILE): references.bib
