import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import { TabProvider } from '../../context/TabContext';

export default function Layout(props) {
  return (
    <TabProvider>
      <OriginalLayout {...props} />
    </TabProvider>
  );
}