#!/bin/bash

# 인자 확인
if [ -z "$1" ] || [ -z "$2" ]; then
    echo "Usage: ./copy_beh.sh [package_name] [file_name_without_extension]"
    exit 1
fi

echo "cp _sm start"

# 스크립트에 하드코딩된 WORKSPACE_ROOT 경로 설정
if [ -z "$WORKSPACE_ROOT" ]; then
    echo "Error: WORKSPACE_ROOT is not set."
    exit 1
fi
# Python 파일 경로 구성 및 복사
src_sm_py="${WORKSPACE_ROOT}/install/$1/local/lib/python3.10/dist-packages/$1/$2_sm.py"
dest_sm_py="./$1/$1/$2_sm.py"

if [ -f "$src_sm_py" ]; then
    cp "$src_sm_py" "$dest_sm_py"
    echo "cp _sm clear"
else
    echo "Error: Source file $src_sm_py does not exist."
fi

# XML 파일 경로 구성 및 복사
src_xml="${WORKSPACE_ROOT}/install/$1/lib/$1/manifest/$2.xml"
dest_xml="./$1/manifest/$2.xml"

if [ -f "$src_xml" ]; then
    cp "$src_xml" "$dest_xml"
    echo "cp _xml clear"
else
    echo "Error: Source file $src_xml does not exist."
fi
