function [fixed_path, cygwin_home] = pfs_fix_path( path )
% Used to fix paths when running cygwin on Windows. This is an internal
% command. Do not use.

if ~ispc()
    fixed_path = path;
else
    
    [pstatus, tmp] = dos('set CYGWIN_HOME');
    if(pstatus == 1)
        cygwin_home = 'c:\\cygwin';
    else
        [pstatus, cygwin_home] = dos('echo %CYGWIN_HOME%');
        cygwin_home = strtrim(cygwin_home); % to remove the final LF
    end
    
    [pstatus, fixed_path] = dos( [ fullfile( cygwin_home, 'bin', 'cygpath' ) ' ' path] );
    
    fixed_path = strtrim( fixed_path );
end

end