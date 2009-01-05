REM This batch file runs ASDoc to generate documentation for Box2D, for windows
REM To use, update the FLEX_HOME variable to the location of your flex installation, e.g.
 set FLEX_HOME=C:\Program Files\Flex
REM Then run this from it's current location


REM Generate list of files used in place of standard flex template
REM dir /B templates > ExcludeFiles.txt

REM Make a local copy of the flex template files, with replacements as specified
xcopy /E /EXCLUDE:ExcludeFiles.txt "%FLEX_HOME%\asdoc\templates" "templates"

REM Run ASDoc
"%FLEX_HOME%\bin\asdoc" ^
    -source-path=../../Source ^
    -doc-sources=../../Source ^
    -output=../../Docs ^
    -window-title "Box2DFlashAS3 Documentation" ^
    -main-title "Box2DFlashAS3 Documentation" ^
    -package Box2D.Collision            "Contains classes handling detecting collisions." ^
    -package Box2D.Collision.Shapes     "Contains shape classes and corresponding definitions." ^
    -package Box2D.Common               "Contains utility functions." ^
    -package Box2D.Common.Math          "Contains 2D vector utilities." ^
    -package Box2D.Dynamics             "Contains b2World, b2Body and related classes." ^
    -package Box2D.Dynamics.Contacts    "Contains classes for tracking contacts between shapes." ^
    -package Box2D.Dynamics.Joints      "Contains joint classes and corresponding definitions." ^
    -templates-path=templates ^
    2> asdoc_errors.txt