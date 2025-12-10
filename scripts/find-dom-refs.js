const fs = require('fs');
const path = require('path');

function walk(dir, cb) {
  const entries = fs.readdirSync(dir, { withFileTypes: true });
  for (const e of entries) {
    const full = path.join(dir, e.name);
    if (e.isDirectory()) {
      if (e.name === 'node_modules' || e.name === '.git') continue;
      walk(full, cb);
    } else {
      cb(full);
    }
  }
}

const matches = [];
walk(process.cwd(), (file) => {
  if (!file.endsWith('.js') && !file.endsWith('.jsx') && !file.endsWith('.ts') && !file.endsWith('.tsx')) return;
  const content = fs.readFileSync(file, 'utf8');
  if (content.includes('document') || content.includes('window')) {
    matches.push(file);
  }
});

if (matches.length === 0) {
  console.log('No files referencing document/window found.');
} else {
  console.log('Files referencing document/window:');
  matches.forEach(f => console.log(f));
}
