#!/bin/bash
if [ $# -lt 5 ]
then
  echo "Usage:"
  echo "./add_model.sh <name> <width> <height> <depth> </path/to/texture.jpg>"
  exit
fi

NAME=$1
W=$2
H=$3
D=$4
TEXTURE=$(echo $(cd $(dirname "$5") && pwd -P)/$(basename "$5"))

cp -ra model_template models/$NAME
cd models/$NAME
sed -e "s/!!NAME!!/$NAME/g" -i model.config
sed -e "s/!!NAME!!/$NAME/g" -i model.sdf
sed -e "s/!!W!!/$W/g" -i model.sdf
sed -e "s/!!H!!/$H/g" -i model.sdf
sed -e "s/!!D!!/$D/g" -i model.sdf
sed -e "s/!!NAME!!/$NAME/g" -i materials/scripts/template.material
mv materials/scripts/template.material materials/scripts/$NAME.material
cp $TEXTURE materials/textures/$NAME.jpg
echo "object $NAME created"

