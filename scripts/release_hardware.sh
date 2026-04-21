#!/bin/bash

VER="r0.1"
PRJ="ovrdrive"
PRJDIR="ee/ovrdrive/"

cd $PRJDIR
mkdir out/$VER  2>/dev/null
rm -r out/$VER/ 2>/dev/null
mkdir out/$VER/ 2>/dev/null
mkdir out/$VER/fab/ 2>/dev/null
mkdir out/$VER/fab/gerber/ 2>/dev/null
cp readme.txt out/$VER/fab/

kicad-cli pcb export gerbers \
    --layers 'F.Cu,In1.Cu,In2.Cu,B.Cu,F.Fab,B.Fab,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts,F.Paste,B.Paste,User.1,User.9' \
    --output out/$VER/fab/gerber/ \
    --no-x2 \
    --subtract-soldermask \
    $PRJ.kicad_pcb

kicad-cli pcb export drill \
    --output out/$VER/fab/ \
    --format excellon \
    --drill-origin absolute \
    --excellon-units mm \
    --excellon-separate-th \
    $PRJ.kicad_pcb

kicad-cli sch export bom \
    --output out/$VER/fab/bom.csv \
    --fields 'Reference,Value,Footprint,${QUANTITY},MPN' \
    --labels 'Designator,Value,Footprint,Quantity,Part Number' \
    --group-by 'Value,Footprint' \
    --sort-field 'Reference' \
    --sort-asc \
    --exclude-dnp \
    --field-delimiter ',' \
    --string-delimiter '"' \
    --ref-delimiter ',' \
    $PRJ.kicad_sch

kicad-cli pcb export pos \
    --output out/$VER/fab/pos.csv \
    --side both \
    --format csv \
    --units mm \
    --smd-only \
    --exclude-dnp \
    $PRJ.kicad_pcb

kicad-cli sch export pdf $PRJ.kicad_sch -o out/$VER/$PRJ.pdf
kicad-cli pcb export step $PRJ.kicad_pcb -o out/$VER/$PRJ.step
cd out/$VER/

OUT=$PRJ\_$VER.zip
zip -r $OUT fab/*
echo Created release: $VER $OUT
