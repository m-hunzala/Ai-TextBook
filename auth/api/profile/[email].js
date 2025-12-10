// pages/api/auth/profile/[email].js (for Next.js) or equivalent serverless function

import { neon } from '@neondatabase/serverless';

const sql = neon(process.env.NEON_DB_URL || '');

export default async function handler(req, res) {
  if (req.method !== 'GET') {
    return res.status(405).json({ error: 'Method not allowed' });
  }

  const { email } = req.query;

  try {
    // Check if email is provided
    if (!email) {
      return res.status(400).json({ error: 'Email parameter is required' });
    }

    // Fetch user profile by email
    const result = await sql(
      'SELECT * FROM user_profiles WHERE email = $1',
      [email]
    );

    if (result.length === 0) {
      return res.status(404).json({ error: 'User profile not found' });
    }

    // Remove sensitive information before sending
    const { id, name, email: userEmail, ...profile } = result[0];
    const sanitizedProfile = {
      id,
      name,
      email: userEmail,
      ...profile
    };

    res.status(200).json(sanitizedProfile);
  } catch (error) {
    console.error('Error fetching profile:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
}

export const config = {
  api: {
    bodyParser: false,
  },
};