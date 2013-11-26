SVG=$(wildcard *.svg)
PNG=$(wildcard *.png)
PDF=$(SVG:.svg=.pdf)
EPS=$(SVG:.svg=.eps) $(PNG:.png=.eps)


.PHONY: all
all: $(PDF) fp

%.pdf : %.svg
	inkscape --without-gui $< -A $@

%.eps : %.svg
	inkscape --without-gui $< -E $@

%.eps : %.png
	inkscape --without-gui $< -E $@

clean:
	rm -f $(PDF)
	rm -f $(EPS)

show:
	$(info all svg: $(SVG))
	$(info all pdf: $(PDF))
	$(info all eps: $(EPS))

fp:
	cvora_fp.sh
