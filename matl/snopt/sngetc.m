% function  cvalue = sngetc( option )
%     The STRING-VALUED optional parameter defined by the string "option"
%     is assigned to cvalue.
%
%     For a description of all the optional parameters, see the
%     snopt documentation.
%
function cvalue = sngetc( option )

getoptionC = 6;
cvalue = snoptcmex( getoptionC, option );
