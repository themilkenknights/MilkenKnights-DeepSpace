#!/bin/sh
##Run Version separately and change the '3.1.366' to the latest version.
# Grab the libraries -- not sure this is necessary, but can't hurt
[ -d ~/navx-maxp ] || mkdir ~/navx-mxp
cd ~/navx-mxp
rm -f navx-mxp-libs.zip
curl -O https://www.kauailabs.com/public_files/navx-mxp/navx-mxp-libs.zip
unzip -o navx-mxp-libs.zip

# Get the vendor file
mkdir ~/frc2019/vendordeps
cd ~/frc2019/vendordeps
rm -f navx_frc.json
curl -O https://www.kauailabs.com/dist/frc/2019/navx_frc.json 

# Determine the version file the vendor file -- use the second part of the first line, and clean out garbage characters
VERSION=$(grep version navx_frc.json | head -1 | cut -f2 -d: | sed -e 's/[ ",]//g' -e 's/\n//g' -e 's/\r//g' )
echo "Version is: " 3.1.366

echo "CPP Start"


# Install the C++ libraries
cd ~/frc2019/maven/com
mkdir -p kauailabs/navx/frc/navx-cpp
cd kauailabs/navx/frc/navx-cpp
rm -f maven-metadata.xml
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/maven-metadata.xml
rm -rf 3.1.366
mkdir 3.1.366
cd 3.1.366
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366-headers.zip
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366-linuxathena.zip
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366-linuxathenadebug.zip
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366-linuxathenastatic.zip
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366-linuxathenastaticdebug.zip
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366-sources.zip
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-cpp/3.1.366/navx-cpp-3.1.366.pom

echo "CPP Done"

# Install the Java libraries
cd ~/frc2019/maven/com
mkdir -p kauailabs/navx/frc/navx-java
cd kauailabs/navx/frc/navx-java
echo "1"


rm -f maven-metadata.xml
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-java/maven-metadata.xml

echo "2"

rm -rf 3.1.366
mkdir 3.1.366
cd 3.1.366

echo "3"

curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-java/3.1.366/navx-java-3.1.366-javadoc.jar  
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-java/3.1.366/navx-java-3.1.366-sources.jar  
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-java/3.1.366/navx-java-3.1.366.jar 
curl -O https://repo1.maven.org/maven2/com/kauailabs/navx/frc/navx-java/3.1.366/navx-java-3.1.366.pom  

echo "Java Done"


cd ~
