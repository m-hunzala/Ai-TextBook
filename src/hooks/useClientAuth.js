import { useState, useEffect } from 'react';

// Custom hook that only initializes auth on the client-side
const useClientAuth = () => {
  const [authClient, setAuthClient] = useState(null);
  const [session, setSession] = useState(null);
  const [isReady, setIsReady] = useState(false);

  useEffect(() => {
    // Only run on client-side
    if (typeof window !== 'undefined') {
      import('better-auth/react').then(({ useAuthQuery }) => {
        const authQuery = useAuthQuery();
        
        // Set up a listener for session changes
        authQuery.subscribe((state) => {
          setSession(state.data?.session || null);
        });
        
        setAuthClient(authQuery.client);
        setSession(authQuery.data?.session || null);
        setIsReady(true);
      }).catch(err => {
        console.error('Failed to initialize auth client:', err);
        setIsReady(true); // Still mark as ready to avoid hanging
      });
    }
  }, []);

  return { client: authClient, session, isReady };
};

export default useClientAuth;