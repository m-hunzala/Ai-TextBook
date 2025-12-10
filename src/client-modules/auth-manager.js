// Client-side authentication manager

// Check authentication state and update UI accordingly
function updateAuthUI() {
  const isAuthenticated = localStorage.getItem('auth') === 'true';
  const logoutBtn = document.getElementById('logout-btn');
  const loginLink = document.querySelector('a[href="/login"]');

  if (logoutBtn) {
    logoutBtn.style.display = isAuthenticated ? 'inline-flex' : 'none';
  }

  if (loginLink) {
    loginLink.style.display = isAuthenticated ? 'none' : 'inline-flex';
  }

  // Add body class for CSS styling
  if (isAuthenticated) {
    document.body.classList.add('authenticated');
  } else {
    document.body.classList.remove('authenticated');
  }
}

// Handle logout button click
function handleLogout() {
  // Remove authentication from localStorage
  localStorage.removeItem('auth');
  localStorage.removeItem('currentUser');

  // Update UI
  updateAuthUI();

  // Redirect to login page
  window.location.href = '/login';
}

// Initialize authentication management when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
  // Update UI based on current auth state
  updateAuthUI();

  // Add event listener to logout button
  const logoutBtn = document.getElementById('logout-btn');
  if (logoutBtn) {
    logoutBtn.addEventListener('click', handleLogout);
  }

  // Listen for storage changes (in case user logs in/out from another tab)
  window.addEventListener('storage', function(e) {
    if (e.key === 'auth') {
      updateAuthUI();
    }
  });
});

// Export for use in Docusaurus config if needed
export { updateAuthUI, handleLogout };