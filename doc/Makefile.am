EXTRA_DIST = faq.txt pfs_format_spec.tex pfs_format_spec.pdf data_model.png

TARGET=pfs_format_spec
SOURCES=$(top_srcdir)/doc/pfs_format_spec.tex

# make pdf by default
all: ${TARGET}.pdf

#${TARGET}.bbl: ${TARGET}.bib
#	(pdflatex ${TARGET} || rm ${TARGET}.pdf) && bibtex ${TARGET} && pdflatex ${TARGET}

${TARGET}.pdf: ${SOURCES} ${TARGET}.tex
	@((TEXINPUTS=$(top_srcdir)/doc: && pdflatex ${TARGET} && pdflatex ${TARGET}) || rm ${TARGET}.pdf)

clean:
	rm -f ${TARGET}.{log,aux,ps,dvi,bbl,blg,log,pdf}

view: ${TARGET}.pdf
	acroread ${TARGET}.pdf

PHONY : all clean view
