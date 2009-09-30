REM This batch file runs ASDoc to generate documentation for Box2D, for windows
REM To use, update the FLEX_HOME variable to the location of your flex installation, e.g.
set FLEX_HOME=C:\Program Files\Flex
REM Then run this from it's current location


REM Generate list of files used in place of standard flex template
REM dir /B templates > ExcludeFiles.txt

REM Make a local copy of the flex template files, with replacements as specified
xcopy /Y /E /EXCLUDE:ExcludeFiles.txt "%FLEX_HOME%\asdoc\templates" "templates"

REM Run ASDoc
"%FLEX_HOME%\bin\asdoc" ^
    -source-path=../../Source ^
    -doc-sources=../../Source ^
    -output=../../Docs ^
    -window-title "Box2DFlashAS3 Documentation" ^
    -main-title "Box2DFlashAS3 Documentation" ^
    -templates-path=templates ^
    2> asdoc_errors.txt
REM Add the following for a nice footer
REM -footer "Box2DFlashAS3 r37"