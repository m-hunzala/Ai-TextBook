import React from 'react';
import ColorModeToggle from '@site/src/components/ColorModeToggle';

const ColorModeToggleComponent = (props) => {
  return (
    <div style={{ display: 'flex', alignItems: 'center', height: '100%' }}>
      <ColorModeToggle {...props} />
    </div>
  );
};

export default ColorModeToggleComponent;