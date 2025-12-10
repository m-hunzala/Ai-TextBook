import React from 'react';
import { UserProvider } from './contexts/UserContext';

export default function App({ children }) {
  return (
    <UserProvider>
      {children}
    </UserProvider>
  );
}