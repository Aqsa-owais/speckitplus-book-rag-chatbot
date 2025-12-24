import React from 'react';
import OriginalDocSidebar from '@theme-original/DocSidebar';

// Simple wrapper that passes through to the original DocSidebar
// We'll handle module-specific content filtering at the page level instead
const DocSidebar = (props) => {
  return (
    <OriginalDocSidebar {...props} />
  );
};

export default DocSidebar;