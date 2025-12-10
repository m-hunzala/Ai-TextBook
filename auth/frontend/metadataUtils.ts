// Frontend utility functions for user metadata fetching

interface UserMetadata {
  id: string;
  userId: string;
  softwareBackground?: string;
  hardwareAvailable?: string[];
  experienceLevel?: string;
  createdAt: string;
  updatedAt: string;
}

// Get current user's profile with metadata
export const getCurrentUserProfile = async (): Promise<{ user: any; metadata: UserMetadata } | null> => {
  try {
    const token = localStorage.getItem('auth-token');
    if (!token) {
      throw new Error('No authentication token found');
    }

    const response = await fetch('/api/auth/profile', {
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
    });

    if (!response.ok) {
      throw new Error('Failed to fetch user profile');
    }

    const data = await response.json();
    return data;
  } catch (error) {
    console.error('Error fetching user profile:', error);
    throw error;
  }
};

// Update user's metadata
export const updateUserMetadata = async (metadata: Partial<UserMetadata>): Promise<UserMetadata> => {
  try {
    const token = localStorage.getItem('auth-token');
    if (!token) {
      throw new Error('No authentication token found');
    }

    const response = await fetch('/api/auth/profile', {
      method: 'PUT',
      headers: {
        'Authorization': `Bearer ${token}`,
        'Content-Type': 'application/json',
      },
      body: JSON.stringify(metadata),
    });

    if (!response.ok) {
      throw new Error('Failed to update user metadata');
    }

    const data = await response.json();
    return data.metadata;
  } catch (error) {
    console.error('Error updating user metadata:', error);
    throw error;
  }
};

// Example usage in a React component
export const useUserMetadata = () => {
  const [userProfile, setUserProfile] = React.useState<any>(null);
  const [loading, setLoading] = React.useState(true);
  const [error, setError] = React.useState<string | null>(null);

  React.useEffect(() => {
    const fetchUserProfile = async () => {
      try {
        setLoading(true);
        const profile = await getCurrentUserProfile();
        setUserProfile(profile);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Unknown error');
      } finally {
        setLoading(false);
      }
    };

    fetchUserProfile();
  }, []);

  return { userProfile, loading, error, refetch: () => {
    // Refetch function
    const fetchUserProfile = async () => {
      try {
        setLoading(true);
        const profile = await getCurrentUserProfile();
        setUserProfile(profile);
      } catch (err) {
        setError(err instanceof Error ? err.message : 'Unknown error');
      } finally {
        setLoading(false);
      }
    };
    fetchUserProfile();
  }};
};

// Example of how to use the metadata in a React component
const UserProfileExample: React.FC = () => {
  const { userProfile, loading, error } = useUserMetadata();

  if (loading) return <div>Loading profile...</div>;
  if (error) return <div>Error: {error}</div>;

  if (!userProfile) return <div>Not logged in</div>;

  return (
    <div>
      <h2>User Profile</h2>
      <p><strong>Name:</strong> {userProfile.user.name}</p>
      <p><strong>Email:</strong> {userProfile.user.email}</p>
      <p><strong>Software Background:</strong> {userProfile.user.softwareBackground || 'Not provided'}</p>
      <p><strong>Hardware Available:</strong> {userProfile.user.hardwareAvailable?.join(', ') || 'Not specified'}</p>
      <p><strong>Experience Level:</strong> {userProfile.user.experienceLevel || 'Not specified'}</p>
    </div>
  );
};

// For Docusaurus integration, you'd use something like this in a React component:
const DocusaurusAuthExample: React.FC = () => {
  const [isLoggedIn, setIsLoggedIn] = React.useState(false);
  const [userMetadata, setUserMetadata] = React.useState<UserMetadata | null>(null);

  React.useEffect(() => {
    // Check if user is authenticated on component mount
    checkAuthStatus();
  }, []);

  const checkAuthStatus = async () => {
    try {
      const profile = await getCurrentUserProfile();
      if (profile) {
        setIsLoggedIn(true);
        setUserMetadata({
          softwareBackground: profile.user.softwareBackground,
          hardwareAvailable: profile.user.hardwareAvailable,
          experienceLevel: profile.user.experienceLevel,
        } as UserMetadata);
      }
    } catch (error) {
      setIsLoggedIn(false);
    }
  };

  if (!isLoggedIn) {
    return (
      <div>
        <p>Please sign in to access personalized content.</p>
        {/* You would integrate the AuthComponent here */}
      </div>
    );
  }

  return (
    <div>
      <h3>Personalized for you:</h3>
      <p>Based on your experience level: <strong>{userMetadata?.experienceLevel}</strong></p>
      {userMetadata?.hardwareAvailable && userMetadata.hardwareAvailable.length > 0 && (
        <p>Your hardware: {userMetadata.hardwareAvailable.join(', ')}</p>
      )}
    </div>
  );
};

// Note: You would need to declare React in a real implementation
// import React from 'react';