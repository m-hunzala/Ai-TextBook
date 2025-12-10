// This file patches the child_process module to fix the DEP0190 deprecation warning
// by ensuring proper argument escaping when shell: true is used

// The issue occurs when shell commands are constructed unsafely
// This patch intercepts child_process calls and properly escapes arguments

// Save this file as 'shell-safe.js' and use it to wrap your Node.js processes
// Or add it as a preload script using NODE_OPTIONS='--require ./shell-safe.js'

const child_process = require('child_process');
const util = require('util');

// Store original functions
const originalSpawn = child_process.spawn;
const originalExec = child_process.exec;
const originalExecFile = child_process.execFile;

// Safe spawn function
child_process.spawn = function(command, args, options) {
  // If shell is true and we have arguments, we need to properly escape them
  if (options && options.shell === true && Array.isArray(args) && args.length > 0) {
    // On Windows, we need to handle arguments differently
    if (process.platform === 'win32') {
      // Join args with proper escaping for Windows
      const escapedArgs = args.map(arg => {
        if (typeof arg === 'string') {
          // Simple escaping for Windows - wrap in quotes if contains spaces or special chars
          if (/[\s"<>|&^()\[\]{}]/.test(arg)) {
            return '"' + arg.replace(/"/g, '""') + '"';
          }
          return arg;
        }
        return String(arg);
      });
      const fullCommand = command + ' ' + escapedArgs.join(' ');
      return originalSpawn(fullCommand, { ...options, shell: true });
    } else {
      // For Unix-like systems, ensure arguments are properly quoted
      const escapedArgs = args.map(arg => {
        if (typeof arg === 'string') {
          // Use shell escaping - wrap in single quotes and escape any single quotes within
          return "'" + arg.replace(/'/g, "'\"'\"'") + "'";
        }
        return String(arg);
      });
      const fullCommand = command + ' ' + escapedArgs.join(' ');
      return originalSpawn(fullCommand, { ...options, shell: true });
    }
  }
  
  return originalSpawn(command, args, options);
};

// Safe exec function
child_process.exec = function(command, options, callback) {
  // For exec, the command should already be a properly formatted string
  return originalExec(command, options, callback);
};

// Safe execFile function
child_process.execFile = function(file, args, options, callback) {
  if (Array.isArray(args) && options && options.shell === true) {
    // If shell is true with execFile, handle the arguments safely
    const escapedArgs = args.map(arg => {
      if (typeof arg === 'string') {
        if (process.platform === 'win32') {
          if (/[\s"<>|&^()\[\]{}]/.test(arg)) {
            return '"' + arg.replace(/"/g, '""') + '"';
          }
        } else {
          return "'" + arg.replace(/'/g, "'\"'\"'") + "'";
        }
      }
      return String(arg);
    });
    
    const fullCommand = file + ' ' + escapedArgs.join(' ');
    return originalExec(fullCommand, options, callback);
  }
  
  return originalExecFile(file, args, options, callback);
};

console.log('Shell-safe patch applied to prevent DEP0190 deprecation warning.');