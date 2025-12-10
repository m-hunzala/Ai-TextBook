import React, { useState, useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import useClientAuth from '@site/src/hooks/useClientAuth';
import { useLocation } from '@docusaurus/router';
import ChatWidget from '@site/src/theme/ChatWidgetWrapper';

import './layout.css';

const Layout = (props) => {
  const { client: authClient, session, isReady } = useClientAuth();
  const [showUserMenu, setShowUserMenu] = useState(false);
  const location = useLocation();

  const handleSignOut = async () => {
    if (authClient?.signOut) {
      try {
        await authClient.signOut();
        // Redirect to homepage after sign out
        window.location.href = '/';
      } catch (error) {
        console.error('Sign out error:', error);
      }
    }
  };

  // Close user menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event) => {
      if (showUserMenu && !event.target.closest('.navbar__auth-menu')) {
        setShowUserMenu(false);
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [showUserMenu]);

  return (
    <>
      <OriginalLayout {...props}>
        {/* Removed auth menu (Sign In/Sign Up buttons) */}
        {/*
        <div className="navbar__auth-extensions">
          {isReady && !session && location.pathname !== '/signin' && location.pathname !== '/signup' ? (
            <div className="navbar__items navbar__items--right auth-menu-unauthed">
              <a href="/signin" className="navbar__item navbar__link navbar__auth-link">
                Sign In
              </a>
              <a href="/signup" className="navbar__item navbar__link button button--primary navbar__auth-button">
                Sign Up
              </a>
            </div>
          ) : isReady && session ? (
            <div className="navbar__items navbar__items--right auth-menu-authed">
              <div className="navbar__item user-menu-container">
                <button
                  className="user-menu-button"
                  onClick={() => setShowUserMenu(!showUserMenu)}
                  aria-expanded={showUserMenu}
                  aria-haspopup="true"
                >
                  <span className="user-menu-text">
                    {session.user?.name || session.user?.email.split('@')[0]}
                  </span>
                  <span className="user-menu-icon" aria-hidden="true">▼</span>
                </button>

                {showUserMenu && (
                  <div className="user-menu-dropdown" role="menu">
                    <div className="user-menu-header">
                      <div className="user-menu-name">{session.user?.name}</div>
                      <div className="user-menu-email">{session.user?.email}</div>
                    </div>
                    <hr className="user-menu-divider" />
                    <button
                      className="user-menu-item"
                      onClick={handleSignOut}
                      role="menuitem"
                    >
                      Sign Out
                    </button>
                  </div>
                )}
              </div>
            </div>
          ) : null}
        </div>
        */}
        {props.children}
        {/* Add the chat widget to all pages */}
        <ChatWidget />
      </OriginalLayout>
    </>
  );
};

export default Layout;