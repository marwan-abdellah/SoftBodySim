#!/bin/bash

proj="symulacja_ciala_miekkiego_z_wyk_cuda"

pdflatex $proj.tex
bibtex $proj
pdflatex $proj.tex
pdflatex $proj.tex
evince $proj.pdf

