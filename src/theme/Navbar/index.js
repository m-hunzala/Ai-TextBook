import React from 'react';
import Navbar from '@theme-original/Navbar';
import AuthManager from '@site/src/components/AuthManager';

import './AuthNavbar.css';

const AuthNavbar = (props) => {
  return (
    <Navbar {...props}>
      {/* This will be visible in the main navbar on desktop */}
      <div className="navbar__auth-manager">
        <AuthManager />
      </div>
    </Navbar>
  );
};

export default AuthNavbar;