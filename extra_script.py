#Import("env")

fr = open('.pio/build/esp32doit-devkit-v1/VERSION.html','r')
versionFW=fr.read()
fr.close()

versionFW = float(versionFW)+ .01

fw = open('.pio/build/esp32doit-devkit-v1/VERSION.html','w')
print(versionFW, file=fw)
fw.close()

f = open('include/version.h','w')
print('#define actualVersion ', versionFW, file=f)
f.close()

#env.Replace(PROGNAME="firmware_%s" % env.GetProjectOption("VERSION"))