rm /tmp/libs
for lib in *.dylib; do otool -L $lib | grep -v @executable_path | cut -f 1 -d '(' >> /tmp/libs; echo "======"; done
cat /tmp/libs |sort |uniq
