// pages/api/auth/session.js (for Next.js) or equivalent serverless function

import { auth } from '../../../auth/auth.config'; // Adjust path as needed
import { neon } from '@neondatabase/serverless';

const sql = neon(process.env.NEON_DB_URL || '');

export default async function handler(req, res) {
  // First, authenticate with Better Auth
  const session = await auth.api.getSession({
    headers: req.headers,
  });

  if (!session) {
    return res.status(401).json({ error: 'Unauthorized' });
  }

  try {
    // Fetch user profile from our custom table
    const userProfile = await sql(
      'SELECT * FROM user_profiles WHERE email = $1',
      [session.user.email]
    );

    if (userProfile.length === 0) {
      return res.status(404).json({ error: 'User profile not found' });
    }

    // Create a custom session object with user profile
    const customSession = {
      user: {
        id: session.user.id,
        email: session.user.email,
        name: session.user.name,
        ...userProfile[0] // Include all profile fields
      },
      expires: session.expiresAt
    };

    // In a real implementation, you'd set an app-specific session cookie here
    // For now, we'll just return the session data
    res.status(200).json(customSession);
  } catch (error) {
    console.error('Error fetching user profile:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
}

export const config = {
  api: {
    bodyParser: true,
  },
};