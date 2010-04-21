import os, string, re

def mos_debug_preprocessor(target, source, env):
    
    myfile = open(str(source[0]), 'r+')
    
    myarray = []
    myflag = False
    append_flag = False
    return_flag = False
    
    for inLine in myfile.readlines():


### prefix insertions


        # procedure returns
        
        if(return_flag):
            if(re.match("}\s*\n", inLine)):
                if(env['dbcode_procedure']):
                    myarray.append("   // prodecure exit debugging code //\n")
                    myarray.append("   mos_debug_set_trace(DBCODE_PROCEDURE_RETURN);\n")
                    return_flag = False
                    append_flag = True

        # any whitespace, return keyword, any whitespace, any characters, semicolon, any whitespace, newline
        if(re.match("\s*return.*;[^\\\]*\n", inLine)):
            if(env['dbcode_procedure']):
                myarray.append("   // prodecure exit debugging code //\n")
                myarray.append("{\n   mos_debug_set_trace(DBCODE_PROCEDURE_RETURN);\n" + inLine.replace('\r','') + "}\n")
                append_flag = True
######
        else:   
            # remove the windows return character (^M)
            myarray.append(inLine.replace('\r',''))

#ONLY UNCOMMENT THIS IF ABOVE IS MODIFIED
#        myarray.append(inLine.replace('\r',''))

            
#postfix insertions

        # procedure calls
                
        if(myflag):
            if(re.search("{",inLine)):
                # code to insert at entry
                myarray.append("   // procedure entry debugging code //\n")
                myarray.append("   mos_debug_check_stack();\n")
                if(env['dbcode_procedure']):
                    myarray.append("   mos_debug_set_trace(DBCODE_PROCEDURE_CALL);\n")
                append_flag = True
                myflag = False

        # optional 'static' keyword followed by any whitespace
        # a typename, optional *, any whitespace, optional *the declaration, any whitespace,
        # open paren, any characters for parameters, close paren, any whitespace,
        # newline
        if(re.match("(static\s+)?\w+[ \t\n\r\f\v\*\&]+\w+\s*\(.*\)\s*\n", inLine)):

            # if the { character will either be on this line or following
            if(re.search("{",inLine)):
                # code to insert at entry
                myarray.append("   // procedure debugging code //\n")
                myarray.append("   mos_debug_check_stack();")
                if(env['dbcode_procedure']):
                    myarray.append("   mos_debug_set_trace(DBCODE_PROCEDURE_CALL);")
                append_flag = True
            else:
                myflag = True
            return_flag = True
                
######

    myfile.close()

    if append_flag == True:
        myarray.insert(0, "#include \"mos_debugging.h\"\n")

    myfile = open(str(target[0]),'w+')

    for outLine in myarray:
        #print outLine
        myfile.write(outLine)

    myfile.close()
